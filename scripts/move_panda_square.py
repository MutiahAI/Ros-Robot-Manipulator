#!/usr/bin/env python
# Code written by Mutiah Apampa

import rospy
import sys
import copy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from math import pi
from ar_week10_test.msg import square_params


class SquarePlanner:
    def __init__(self):
        # Initialise node
        rospy.init_node('square_planner', anonymous=True)

        # subscribe to cubic_traj_params and send data to callback
        rospy.Subscriber('square_params', square_params, self.callback)

        # Moveit_commander initialization
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot_command = moveit_commander.RobotCommander()

        # Moveit Planning Scene Interface initialization
        self.scene = moveit_commander.PlanningSceneInterface()

        # Moveit Move Group Commander intialization
        self.arm = moveit_commander.MoveGroupCommander('panda_arm')

        # Panda Arm robot moveit Display trajectory publisher initialization
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                DisplayTrajectory,
                                                queue_size=20)


    def callback(self, data):
        try:
            # Get data from the square_size_generator topic
            print ('Move Panda - Received square size, s = %s ' % data.length)
            print('----------------------------------------------------------')

            # Getting the Start configuration
            print ('Move Panda - Going to Start Configuration')
            print('----------------------------------------------------------')
            start_conf = [0, -pi / 4, 0, -pi / 2, 0, pi / 3, 0]

            # Set the Panda arm to the given start configuration
            self.arm.go(start_conf, wait=True)

            # Set a time delay to give threads time on the processor
            rospy.sleep(2)

            # Ensuring there is no residual movement
            self.arm.stop()

            # Planning Motion Trajectory
            print ('Move Panda - Planning Motion Trajectory')
            print('----------------------------------------------------------')

            # Waypoints initialization
            waypoints = []

            # Get the current group positions
            pos = self.arm.get_current_pose().pose

            # First move sideways (y)
            pos.position.y += data.length  
            waypoints.append(copy.deepcopy(pos))

            # Second move forward/backwards in (x)
            pos.position.x += data.length  
            waypoints.append(copy.deepcopy(pos))

            # Third move sideways (y)
            pos.position.y -= data.length  
            waypoints.append(copy.deepcopy(pos))

            # Forth move forward/backwards in (x)
            pos.position.x -= data.length  
            waypoints.append(copy.deepcopy(pos))

            (plan, fraction) = self.arm.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.01,  # eef_step 
                0.0)  # jump_threshold 

            # Set a time delay to give threads time on the processor
            rospy.sleep(3)

            # Displaying a trajectory
            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot_command.get_current_state()
            display_trajectory.trajectory.append(plan)
            
            print ('Move Panda - Showing Planned Trajectory')
            print('----------------------------------------------------------')
            self.display_trajectory_publisher.publish(display_trajectory)

            # Set a time delay to give threads time on the processor
            rospy.sleep(3)

            # Executing planned trajectory
            print ('Move Panda - Executing Planned Trajectory')
            print('----------------------------------------------------------')
            self.arm.execute(plan, wait=True)

            rospy.sleep(2)

            # Waiting for square trajectory
            print ('Move Panda - Waiting for desired size of square trajectory')
            print('----------------------------------------------------------')

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        node = SquarePlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
