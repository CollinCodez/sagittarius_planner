# **Sagittarius K1 Robotic Arm Motion Planner**

This repository contains a Python-based motion planning script for the **Sagittarius 6-DOF robotic arm**. The project uses **ROS** and **MoveIt** for precise and collision-free motion planning, enabling the robotic arm to plan and execute complex tasks.

## **Features**
- **Joint and Pose Goals**: Enables precise control of the arm's joints and end-effector positions.
- **Collision-Free Planning**: Plans safe trajectories in cluttered environments.
- **RViz Integration**: Visualize motion plans in real-time.
- **Extensibility**: Easily adapt the planner for additional motion tasks or new robotic configurations.

---

## **Prerequisites**

Ensure you have the following installed and configured:
1. **ROS Melodic** on Ubuntu 20.04.
2. **MoveIt Framework**.
3. **Python 3** with necessary ROS Python libraries.
4. A properly set up **Sagittarius workspace (`sagittarius_ws`)**. Instructions for setting up the workspace can be found at [NXROBO's Sagittarius Workspace Repository](https://github.com/NXROBO/sagittarius_ws).
5. The **Flask** Python library

---

## **Installation**

Please make sure you have installed your "sagittarius_ws" workspace from https://github.com/NXROBO/sagittarius_ws

Copy the **planner.py** file and **WebServer** folder to the following path: ```~/sagittarius_ws/src/sagittarius_arm_ros/sagittarius_moveit```

Verify that the planner.py file has been copied in sagittarius_moveit directory

In **planner.py**, edit the file path in the `sys.path.insert(` line near the top of the file to appropriatly match the path to the WebServer folder on your computer.

In **WebServer.py**, edit the `host` in the `startFlask()` function to match the IP address or hostname you want the web server to run on.

To update the ROS launch files, open a new terminal and run the following commands:

```
cd ~/sagittarius_ws

catkin_make
```



## **Running the Program**

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

---
## **Using the Web Page**
Navigate to the appropriate IP address & Port in a web browser.

The **Mode** buttons work as follows:
- **Stop** should stop the arm from running whatever it is currently doing ***NOTE:*** *This may only stop at the end of the current cycle*
- **Auto** will make the arm begin the automatic cycle for handing out candy
- **Manual** will enter a mode to allow you to manually adjust the joint positions using the sliders ***NOTE:*** *The sliders do not move to match where the arm currently is. They only change with inputs.*
- **Exit** will cleanly exit the program


The **Position** Buttons are not fully implemented at the moment, and will likely not do anything. This is also inteded to work during the **Manual** mode.
