# Robust Trajectory Tracking for Quadrotor using Sliding Mode Control

The objective of this project is to develop a robust control scheme to enable a quadrotor to track
desired trajectories in the presence of external disturbances.The control design is tested on the Crazyflie 2.0 platform. Crazyflie is a quadrotor
that is classified as a micro air vehicle (MAV), as it only weighs 27 grams and can fit in a hand.

# Crazyflie 2.0 Setup in Gazebo

To set up the Crazyflie 2.0 quadrotor in Gazebo, we need to install additional ROS dependencies
for building packages as below:
```
sudo apt update
```
```
sudo apt install ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink
```
```
sudo apt install ros-noetic-octomap-mapping ros-noetic-control-toolbox
```
```
sudo apt install python3-vcstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev
```
```
rosdep update
```
```
sudo apt-get install ros-noetic-ros libgoogle-glog-dev
```
# Instruction for setting the Enviroment 

**Clone this repo to create a new ROS workspace for Sliding Mode Controller**

```
git clone https://github.com/PurvangPatel/Robust_Trajectory_Tracking_for_Quadrotor_using_Sliding_Mode_Control.git
```
**Initialize catkin workspace**

```
cd ~/Robust_Trajectory_Tracking_for_Quadrotor_using_Sliding_Mode_Control/src
```

```
catkin_init_workspace
```
```
cd ~/Robust_Trajectory_Tracking_for_Quadrotor_using_Sliding_Mode_Control
```
```
catkin init
```

**Downloading necessary the ROS packages:**

```
cd ~/Robust_Trajectory_Tracking_for_Quadrotor_using_Sliding_Mode_Control/src
```
```
git clone -b dev/ros-noetic https://github.com/gsilano/CrazyS.git
```
```
git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
```

**We need to build the project workspace using python_catkin_tools, therefore we need to configure it:**
```
cd ~/Robust_Trajectory_Tracking_for_Quadrotor_using_Sliding_Mode_Control
```
```
rosdep install --from-paths src -i
```
```
rosdep update
```
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
```
```
catkin build
```
# Running the Project

**Before running the project, make sure to change the directory to store log file for visualization**

src/controller/scripts/**sliding_mode_control.py** -> Change directory on line **243**

src/controller/scripts/**visualization.py** -> Change directory on line **20** and **26**

**To spawn the quadrotor in Gazebo, we can run the following launch file:**

```
roslaunch rotors_gazebo crazyflie2_without_controller.launch
```
**To start the controller, we can run the following script file:**
```
rosrun controller sliding_mode_control.py
```

# Simulation Results

**Trajectory tracking**

Following plot displays the desired trajectory to be tracked in green and the actual trajectory followed by the quadcopter in blue and their deviation.

![trajectory](https://user-images.githubusercontent.com/72921304/208376503-250e9ea8-881f-4274-8a5b-c75674691157.png)

**Gazebo simulation**

![ezgif com-gif-maker](https://user-images.githubusercontent.com/72921304/208376568-bcdbbdfb-691d-4f91-98c8-67eb480cf2fb.gif)
