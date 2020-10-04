#!/usr/bin/env python

# DD2410 - Project Mobile Manipulation
# E-level State Machine
# Matthew Norström 970313, Marcus Jirwe 960903

import numpy as np
from numpy import linalg as LA

import rospy
import sys
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class StateMachine(object):
    def __init__(self):
        
        self.node_name = "Student SM"

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.pick_pose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')

        # This is given in the E-level state machine
        self.cube_location = rospy.get_param(rospy.get_name() + '/cube_pose')

        # Subscribe to topics
        # No topics to suscribe to neccessary in the E-level state machine

        # Wait for service providers
        rospy.wait_for_service(self.pick_srv_nm, timeout = 30)
        rospy.wait_for_service(self.place_srv_nm, timeout = 30)

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.aruco_pose_pub = rospy.Publisher(self.aruco_pose_top, PoseStamped, queue_size=10)

        # Set up action clients
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)
        rospy.sleep(1)


        # Init state machine
        self.state = 0
        rospy.sleep(3)
        self.e_level_state_machine()
        sys.exit(0)

    def e_level_state_machine(self):
        # A simple state machine connecting topics and the like to solve the E-level problem of picking up 
        # and placing the cube on the other table.

        while not rospy.is_shutdown() or (self.state != -1 or self.state != -2): # -1 is supposed to be the state where the robot is finished.
            '''
            1. Complete picking task

            2. Carry cube to second table
                2.1 Rotate robot 180 degrees / pi radians

                2.2 Move to target.

            3. Complete placing task
            '''
            
            # State 0:  Tuck arm. Robot starts out standing in correct position to pick up cube. Thus we move arm first.
            if self.state == 0:
                rospy.loginfo("%s: Tucking the arm...", self.node_name)
                goal = PlayMotionGoal()
                # this is a predefined set of actions available for both Tiago Titanium and Tiago Steel
                goal.motion_name = 'home' 
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                success_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))

                if success_tucking:
                    rospy.loginfo("%s: Arm tuck: ", self.play_motion_ac.get_result())
                    self.state = 1
                else:
                    self.play_motion_ac.cancel_goal()
                    rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
                    self.state = -2 # -2 is the error handling state. Something has gone wrong.
                rospy.sleep(3)

            # state 1: Calling the pick service after publishing the cube location to pick up the cube.
            if self.state == 1:
                try:
                    rospy.loginfo("%s: Attempting to pick up cube...", self.node_name)

                    # Define message with cube location, which is provided for us in the launch file, albait slightly off
                    cube_msg = make_target_message(self.cube_location, PoseStamped())
                    self.aruco_pose_pub.publish(cube_msg)

                    # Making a service for picking up the cube 
                    pick_up_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)
                    pick_up_request = pick_up_srv()

                    if pick_up_request.success == True:
                        print('Cube is grabbed!')
                        self.state = 2
                    else:
                        self.play_motion_ac.cancel_goal()
                        rospy.loginfo("%s: Pick up failed!", self.node_name)
                        self.state = -2
                    rospy.sleep(3)

                except rospy.ServiceException, e:
                    # in the case of error, cancel the goal and set state to "error"
                    self.play_motion_ac.cancel_goal()
                    self.state = -2
                    print('Service call to pick_up_srv failed: %s' %e)

            # State 2:  Move the robot to the other desk.
            if self.state == 2:
                # In the E-level state machine, it's enough to manually control
                # the robot to move to the other table


                # Rotate the robot
                move_msg = Twist()
                move_msg.angular.z = -1
                move_msg.linear.x = 0

                rate = rospy.Rate(10)
                converged = False
                cnt = 0
                rospy.loginfo("%s: Rotate robot", self.node_name)
                # 30 ticks of movement is enough to make it spin 180 degrees
                while not rospy.is_shutdown() and cnt < 30:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                rospy.sleep(3)

                # Stop the robot, by sending a blank move message

                move_msg = Twist()
                self.cmd_vel_pub.publish(move_msg)

                # Drive the robot forwards
                move_msg.linear.x = 1
                move_msg.angular.z = 0
                cnt = 0
                # 10 ticks is enough to make it travel to the other table without a severe impact
                while not rospy.is_shutdown() and cnt < 10:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                # Setting the next state
                self.state = 3
                rospy.sleep(3)

                # Stop the robot, with another empty message (might be superflous, but just in case)
                move_msg = Twist()
                self.cmd_vel_pub.publish(move_msg)
            
            # State 3: Calling the place service after publishing the cube location to pick up the cube.
            if self.state == 3:
                # using the same relative position (to the robot and not map) as we found the cube in, 
                # place the cube.
                cube_msg = make_target_message(self.cube_location, PoseStamped())
                self.aruco_pose_pub.publish(cube_msg)

                place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)
                place_request = place_srv()

                if place_request.success:
                    self.state = -1
                    rospy.loginfo('%s: Cube successfully placed!', self.node_name)

                else:
                    self.state = -2
                    rospy.loginfo('%s: Cube was not successfully placed!', self.node_name)

                rospy.sleep(3)
        
            # The "completed" state, if this is reached, then we are done.
            if self.state == -1:
                print('Done!')
                rospy.signal_shutdown("The mission... The nightmares... They're finally over.")
                return

            # Error handling
            if self.state == -2:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return
        


def make_target_message(target_location, message_type):
    # Function that takes in the cube location and returns a PoseStamped message with the same info.
    # A nice function to have, since we have been given the hardcoded location of the cube in the 
    # E-level state machine, we just need to transform the message into the right type for later.

    target_message = message_type

    target_location_list = make_double_list(target_location)
    # Sets position of cube
    target_message.pose.position.x = target_location_list[0]
    target_message.pose.position.y = target_location_list[1]
    target_message.pose.position.z = target_location_list[2]

    # Sets quaternion orientation of cube
    target_message.pose.orientation.x = target_location_list[3]
    target_message.pose.orientation.y = target_location_list[4]
    target_message.pose.orientation.z = target_location_list[5]
    target_message.pose.orientation.w = target_location_list[6]

    #target_message.header.frame_id = 'odom'

    target_message.header.seq = 1
    target_message.header.stamp = rospy.Time.now()
    target_message.header.frame_id = "base_footprint" # Should this be base_footprint?

    return target_message

def make_double_list(target_location):
    # Since the target_location is a string, we need to split it into floats.

    #split by the comma.
    target_location += ','

    tmp_str = ''
    result_list = []
    for i in target_location:
        if i != ',':
            tmp_str += i
        else:
            result_list.append(float(tmp_str))
            tmp_str = ''

    return result_list

if __name__ == "__main__":
    # the main function, as given
	rospy.init_node('main_state_machine')
	try:
		StateMachine()
	except rospy.ROSInterruptException:
		pass
	rospy.spin()