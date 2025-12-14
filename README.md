# Robotics Project (SLAM on AGILEX Limo Pro)

## 1) Cloning the Repository 

```bash
git clone --recursive https://github.com/nourabouzeid/limo_ws.git
```

If you already cloned the repository without the --recursive flag, initialize the submodules manually:
```bash
git submodule update --init --recursive
```

## 2) Building the Workspace
```bash
cd limo_ws
catkin_make

# always source when opening a new terminal
source devel/setup.bash
```
## 3) Running Simulator and Explorer
```bash
# Simulator
roslaunch limo_gazebo_sim limo_four_diff.launch

# Explorer
roslaunch limo_explore explore.launch

# Visual_search 
roslaunch limo_visual_search visual_search.launch
```