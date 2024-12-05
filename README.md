# **Sagittarius K1 Robotic Arm Motion Planner**

This repository contains a Python-based motion planning script for the **Sagittarius 6-DOF robotic arm**. The project leverages **ROS** and **MoveIt** for precise and collision-free motion planning, enabling the robotic arm to execute complex tasks efficiently.

## **Features**
- **Joint and Pose Goals**: Enables precise control of the arm's joints and end-effector positions.
- **Collision-Free Planning**: Plans safe trajectories in cluttered environments.
- **RViz Integration**: Visualize motion plans in real-time.
- **Extensibility**: Easily adapt the planner for additional motion tasks or new robotic configurations.

---

## **Prerequisites**

Ensure you have the following installed and configured:
1. **ROS Kinetic or Melodic** on Ubuntu 16.04 or 18.04.
2. **MoveIt Framework**.
3. **Python 3** with necessary ROS Python libraries.
4. A properly set up **Sagittarius workspace (`sagittarius_ws`)**. Instructions for setting up the workspace can be found at [NXROBO's Sagittarius Workspace Repository](https://github.com/NXROBO/sagittarius_ws).

---

## **Installation**

Please make sure you have installed your "sagittarius_ws" workspace from https://github.com/NXROBO/sagittarius_ws

Copy the "planner.py" file to the following path: ```~/sagittarius_ws/src/sagittarius_arm_ros/sagittarius_moveit```

Verify that the planner.py file has been copied in sagittarius_moveit directory



In a new terminal, enter the following commands to launch moveit with the sagittarius arm: 

```
cd ~/sagittarius_ws

source devel/setup.bash

roslaunch sagittarius_moveit demo.launch
```


In another terminal, run the following commands to run planner.py python script:
```
cd ~/sagittarius_ws

source devel/setup.bash

cd src/sagittarius_arm_ros/sagittarius_moveit/

ROS_NAMESPACE=sgr532 rosrun sagittarius_moveit planner.py
```


The planning should start now. Follow the on screen instructions in the second terminal and press enter whenever prompted.
