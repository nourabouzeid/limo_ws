# Robotics Project (SLAM on AGILEX Limo Pro)

## Cloning the Repository 

```bash
git clone --recursive <your_repo_url>
```

### If you already cloned the repository without the --recursive flag, initialize the submodules manually:
```bash
git submodule update --init --recursive
```

## Building the Workspace
```bash
cd limo_ws
catkin_make

# always source when opening a new terminal
source devel/setup.bash
```
## Running Simulator and Explorer
```bash
# Simulator
roslaunch limo_gazebo_sim limo_four_diff.launch

# Explorer
roslaunch limo_explore explore.launch
```