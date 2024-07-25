# RBE521-Quadruped-Robot
This repository contains all of the coding efforts required for the final project for RBE521 Legged Robotics. Contributors include Scott Pena, Tim DeMaro, and Nate Dixon.
# How to run our code
NOTE: Make sure roscore is running.\
To launch Spot in Gazebo, run roslaunch spot_micro spot_ai_gazebo.launch
To launch Spot in RViz, run roslaunch spot_micro rviz_spot.launch
Run the following nodes in their own terminal:
1. rosrun spot_control FrontRightLeg.py
2. rosrun spot_control FrontLeftLeg.py
3. rosrun spot_control RearRightLeg.py
4. rosrun spot_control RearLeftLeg.py
5. rosrun spot_control InverseKinematics.py
6. rosrun spot_control trajectories.py (Running this node will immediately start the trajectory generation. Make sure this node runs last.)
