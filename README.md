Instructions:

Please make sure you have created your "sagittarius_ws" directory

Copy the "planner.py" file to the following path: ~/sagittarius_ws/src/sagittarius_arm_ros/sagittarius_moveit

Verify that the planner.py file has been copied in sagittarius_moveit directory



In a new terminal, enter the following commands to launch moveit with the sagittarius arm: 
cd ~/sagittarius_ws
source devel/setup.bash
roslaunch sagittarius_moveit demo.launch


In another terminal, run the following commands to run planner.py python script:
cd ~/sagittarius_ws
source devel/setup.bash
ROS_NAMESPACE=sgr532 rosrun sagittarius_moveit planner.py


The planning should start now. Follow the on screen instructions in the second terminal and press enter whenever prompted.
