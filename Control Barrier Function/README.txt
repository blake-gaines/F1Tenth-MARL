Turtlebot Trajectory Tracking Project - 

Our code is available at https://arcgit.wpi.edu/cibr_jf/mRobot 
The code that we received at the start of the project is contained in the "controller" branch. This branch has files for 1D ACC written in python and 2D ACC written in Matlab

We have pushed all code written by us for implementation on the turtlebot to the branch "yash_shubham". All code written by us can be found on this branch in the folder "mRobot/rajnish_clf_cbf/acc_package/src"

acc_control.py - 1D ACC controller
vicon_publisher.py - Reads input from Vicon and applies median filter for the 1D case

2dmain.py - 2D ACC controller
2dpublisher.py - Reads input from Vicon and applies median filter for the 2D case

plot_bag.py - Utility code for plotting data from rosbags recorded for the 1D case

For running this code on the turtlebot, do the following on the laptop connected to the turtlebot- 
1) In your ROS workspace, install the following packages - Vicon Bridge (https://github.com/ethz-asl/vicon_bridge.git) and ACC controller (branch yash_shubham in repo https://arcgit.wpi.edu/cibr_jf/mRobot) 
2) On the laptop connected to turtlebot, run the command - roslaunch turtlebot_bringup minimal.launch
3) Run the vicon bridge after setting up the ip of the vicon computer - roslaunch vicon_bridge vicon.launch
4)a) For 1D ACC, run the scripts acc_control.py and vicon_publisher.py in the folder "mRobot/rajnish_clf_cbf/acc_package/src"
4)a) For 1D ACC, run the scripts 2dmain.py and 2dpublisher.py in the folder "mRobot/rajnish_clf_cbf/acc_package/src"

Please ensure that proper markers have been set up on the turtlebot and the obstacle so that the Vicon can track it. For compatibility with this code, name the turtlebot object as "turtlebot_traj_track" and the obstacle as "frontcar_traj_track"

Note - The 1D case will run successfuly when the turtlebot travels along the positive y-axis direction of the Vicon system's world frame. 