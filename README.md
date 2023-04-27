### ROS package that automatically generates Cartesian space movements of the end-effector of the Panda robot manipulator: the end-effector will have to "draw" squares of different sizes on the x-y Cartesian plane, starting from a given robot configuration.

#### To run the package:

1.) Download and run the following repositories to run rViz with the Panda Arm Robot in your catkin workspace
	
	$ git clone -b ROS-DISTRO-devel https://github.com/ros-planning/moveit_tutorials.git
	$ git clone -b ROS-DISTRO-devel https://github.com/ros-planning/panda_moveit_config.git
	
	where ROS-DISTRO is the name of the ROS Distribution (eg. melodic, noetic etc.)

2.) Download and unzip the ar_week10_test.zip folder in the src folder of your catkin workspace,

3.) Run the 'catkin_make' command from the same workspace

4.) Run the 'source /opt/ros/noetic/setup.bash'and '. ~/catkin_ws/devel/setup.bash' command from the now opened catkin workspace,

5.) Once the workspace is ready, make all the nodes executable by running the commands below from the "scripts" folder containing all the nodes in the terminal where 'catkin_make' command was run:
  'chmod +x square_size_generator.py
   chmod +x move_panda_square.py'

6.) Run the 'roscore'command,
   
7.) In a separate terminal, launch the Rviz by running:

   'roslaunch panda_moveit_config demo.launch'

8.) Launch the package by running the below in a new terminal:

  'rosrun ar_week10_test square_size_generator.py'
  
9.) In another new terminal, run:

  'rosrun ar_week10_test move_panda_square.py'

10.) Run 'rosrun rqt_plot rqt_plot' in a new terminal to get the visualise the joints positions

### List of Dependencies:

- python 3.8.10
- ros-noetic
- Numpy
- catkin
- ros-kinetic
- rqt
- moveit_commander
- moveit_msgs
