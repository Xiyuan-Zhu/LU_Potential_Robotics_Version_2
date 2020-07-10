LU_Potentail_Robotics_version_2
LU_Potential_Robotics submission for qualifier of IFAC 2020 of F1tenth race car competition by Team LU Potential of Lehigh University, Bethlehem, PA, USA. Team members: Xiyuan Zhu, Andrew Charway, and Yazhou Li.
Team Advisor: Rosa Zheng

The submission contains four files including this README.MD.

Assume that the host computer has installed Ubuntu 18.04, ROS Melodic, and Docker, and the July 2020 version of f1tenth_gym_ros. To run the race in a separate terminal:


1. Git clone the LU-Potential_Robotics_Version_2 package from https://github.com/Lu-Potential/LU_Potential_Robotics_Version_2.git. Copy  
into Lehigh_Ver_2.py and lu_kinetic_version_2.launch into the subfolder /src/f1tenth_gym_ros/scripts/ of your catkin workspace.

2. Make sure that the Lehigh_Ver_2.py file is executable;

3. Create a path in the Home folder called "~/rcws/logs/" and copy the wp-berlin-max-speed-9-2.csv file into the '~/rcws/logs' folder.

4. Under the catkin workspace, source devel/setup.bash, and run:
    $roslaunch f1tenth_gym_ros lu_kinetic_version_2.launch


