#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan # New import for LiDAR data
import math
import time

# --- Configuration Constants ---
LINEAR_SPEED = 0.2      # m/s for recovery move
ANGULAR_SPEED = 1    # rad/s for rotation (collision avoidance)
RECOVERY_DURATION = 3.0 # seconds to move forward or rotate to recover
STALL_THRESHOLD = 10.0  # seconds of no movement before recovery move
MAX_LINEAR_VEL = 0.05   # m/s: Maximum linear velocity magnitude to count as 'not moving'
RATE_HZ = 10            # Hz: Loop rate of the main node

# Collision Avoidance Parameters
OBSTACLE_DISTANCE_THRESHOLD = 1.6 # meters: distance to consider an obstacle imminent
SCAN_ANGLE_RANGE = 0.5           # radians: angular width to check for obstacles (e.g., +/- 28.6 degrees)
FRONTIEER_EMPTY_TIMEOUT = 30.0  # seconds to wait with empty frontiers before terminating

class StallDetector:
    def __init__(self):
        rospy.init_node('stall_detector', anonymous=True)
        
        # --- Publishers and Subscribers ---
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.frontier_sub = rospy.Subscriber('/explore/frontiers', MarkerArray, self.frontier_callback)
        # NEW: Subscriber for Lidar Scan data
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback) 
        
        # --- State Variables ---
        self.last_odom_time = rospy.Time.now()
        self.last_move_time = rospy.Time.now()
        self.is_moving = False
        self.is_recovering = False
        self.last_frontier_time = rospy.Time.now()
        
        # NEW: Safety Flag
        self.imminent_collision = False 
        self.last_scan = None # Store the last scan message

        # Wait for ROS Time to be available
        rospy.loginfo("StallDetector: Waiting for ROS time to start...")
        while rospy.Time.now() == rospy.Time(0):
            rospy.Rate(1).sleep()
        rospy.loginfo("StallDetector: ROS time available. Node initialized.")

        self.rate = rospy.Rate(RATE_HZ)

    # --- Callbacks ---

    def odom_callback(self, msg: Odometry):
        """Processes Odometry messages for stall detection."""
        self.last_odom_time = rospy.Time.now()
        linear_x = msg.twist.twist.linear.x
        linear_y = msg.twist.twist.linear.y
        linear_vel_magnitude = math.sqrt(linear_x**2 + linear_y**2)
        
        if linear_vel_magnitude > MAX_LINEAR_VEL:
            if not self.is_moving:
                self.is_moving = True
            self.last_move_time = self.last_odom_time
        else:
            if self.is_moving:
                self.is_moving = False

    def frontier_callback(self, msg: MarkerArray):
        """Processes Frontier messages to determine exploration status."""
        num_frontiers = len(msg.markers) 
        
        if num_frontiers > 0:
            self.last_frontier_time = rospy.Time.now()
    
    def scan_callback(self, msg: LaserScan):
        """Processes LaserScan messages for collision detection."""
        self.last_scan = msg
        
        # Determine the indices corresponding to the front sector
        if msg.angle_increment == 0 or msg.range_max == 0:
            # Handle bad scan data if necessary
            return

        # Calculate the number of indices that span the desired SCAN_ANGLE_RANGE
        angle_span_indices = int(SCAN_ANGLE_RANGE / msg.angle_increment)
        
        # Get the center index (0 is often the center, but check your LiDAR's configuration)
        # Assuming the scan array is centered (i.e., index 0 is at -180 degrees)
        # If your LiDAR starts at 0 degrees and goes up, adjust this. 
        # For Limo, it's typically centered, so the center is near the end or beginning.
        
        # Let's assume the scan array is 360 degrees, with index 0 at the start.
        # If the scan is centered (e.g., -1.57 to +1.57 radians for 180 degrees), 
        # the center index is half the array length.
        
        # For typical 360-degree scanners, the center index is usually len(ranges)/2
        center_index = len(msg.ranges) // 2 
        
        # Calculate start and end indices for the front sector
        start_index = center_index - angle_span_indices // 2
        end_index = center_index + angle_span_indices // 2

        # Ensure indices are within bounds
        start_index = max(0, start_index)
        end_index = min(len(msg.ranges) - 1, end_index)

        # Check the ranges in the front sector
        min_range_in_front = float('inf')
        
        # Iterate over the front sector
        for i in range(start_index, end_index):
            r = msg.ranges[i]
            # Ignore invalid readings (inf or NaN)
            if not math.isnan(r) and r > msg.range_min and r < msg.range_max:
                min_range_in_front = min(min_range_in_front, r)

        # Check against the collision threshold
        if min_range_in_front < OBSTACLE_DISTANCE_THRESHOLD:
            if not self.imminent_collision:
                rospy.logwarn_throttle(1.0, "SAFETY: Collision imminent (%.2f m). Preventing forward movement.", min_range_in_front)
            self.imminent_collision = True
        else:
            self.imminent_collision = False

    # --- Core Logic ---

    def recovery_move(self):
        self.is_recovering = True
        rospy.logwarn("StallDetector: Stall detected (%.1f s). Initiating recovery move.", STALL_THRESHOLD)
        
        twist_cmd = Twist()
        
        # SAFETY CHECK: If collision is imminent, rotate instead of moving forward
        if self.imminent_collision:
            rospy.logwarn("SAFETY OVERRIDE: Obstacle detected. Initiating %.1f s rotation.", RECOVERY_DURATION)
            twist_cmd.angular.z = ANGULAR_SPEED # Rotate to the left (positive Z)
        else:
            rospy.logwarn("Initiating %.1f s forward linear move.", RECOVERY_DURATION)
            twist_cmd.linear.x = LINEAR_SPEED
        
        start_time = rospy.Time.now()
        
        # Execute the move (forward or rotate) for RECOVERY_DURATION seconds
        while rospy.Time.now() < start_time + rospy.Duration(RECOVERY_DURATION):
            if rospy.is_shutdown():
                break
            self.cmd_vel_pub.publish(twist_cmd)
            self.rate.sleep()
            
        # Stop the robot
        rospy.loginfo("StallDetector: Recovery move complete. Stopping robot.")
        # Send a zero twist command
        self.cmd_vel_pub.publish(Twist()) 
        
        self.is_recovering = False
        self.last_move_time = rospy.Time.now()

    def run(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            
            # 1. Check for Stall Condition
            time_since_move = (current_time - self.last_move_time).to_sec()
            
            if time_since_move > STALL_THRESHOLD and not self.is_recovering:
                # Stall detected: execute recovery action (which includes the safety check)
                self.recovery_move()
            
            # 2. Check for Exploration Completion
            time_since_last_frontier = (current_time - self.last_frontier_time).to_sec()
            
            if time_since_last_frontier > FRONTIEER_EMPTY_TIMEOUT:
                rospy.logwarn("StallDetector: Exploration complete (no active frontiers for %.1f s).", time_since_last_frontier)
                rospy.signal_shutdown("Exploration Complete: Terminating Stall Detector node.")
                break
                
            self.rate.sleep()

if __name__ == '__main__':
    try:
        detector = StallDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass