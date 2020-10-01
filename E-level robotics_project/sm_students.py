#!/usr/bin/env python

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
        '''
        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        '''
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.pick_pose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')

        '''
        self.localization_srv_nm = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        self.clear_cmaps_srv_nm = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        '''

        self.cube_location = rospy.get_param(rospy.get_name() + '/cube_pose')

        # Subscribe to topics
        # Probably subscribe to topic checking gripper status?

        # Wait for service providers
        '''
        rospy.wait_for_service(self.mv_head_srv_nm, timeout = 30)
        '''
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
        '''
        rospy.loginfo("%s: Waiting for move_base action server...", self.node_name)
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /move_base action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to move_base action server", self.node_name)
        '''

        # Init state machine
        self.state = 0
        rospy.sleep(3)
        self.e_level_state_machine()
        sys.exit(0)

    def e_level_state_machine(self):
        '''
        A simple state machine connecting topics and the like to solve the E-level problem of picking up and placing the cube on
        the other table.
        '''

        while not rospy.is_shutdown() or (self.state != -1 or self.state != -2): # -1 is supposed to be the state where the robot is finished.

            '''

            1. Complete picking task

            2. Carry cube to second table
                2.1 Rotate robot 180 degrees / pi radians

                2.2 Move to target.

            3. Complete placing task

            '''
            '''
            # State 0:  Raise robot head service
            if self.state == 0:
            	try:
                    rospy.loginfo("%s: Raising robot head", self.node_name)
                    move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
                    move_head_req = move_head_srv("up")
                    
                    if move_head_req.success == True:
                        self.state = 1
                        rospy.loginfo("%s: Move head up succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Move head up failed!", self.node_name)
                        self.state = -2

                    rospy.sleep(3)
                
                except rospy.ServiceException, e:
                    print "Service call to move_head server failed: %s"%e
            '''
            

            # State 0:  Tuck arm. Robot starts out standing in correct position to pick up cube. Thus we move arm first.
            if self.state == 0:

                # Move this to later in the code? When we move?
                '''
                localization_srv = rospy.ServiceProxy(self.localization_srv_nm, SetBool)
                localization_req = localization_srv()

                clear_costmaps_srv = rospy.ServiceProxy(self.clear_cmaps_srv_nm, Empty)
                clear_costmaps_request = clear_costmaps_srv()
                '''

                rospy.sleep(1)

                rospy.loginfo("%s: Tucking the arm...", self.node_name)
                goal = PlayMotionGoal()
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
                    # Define message with cube location

                    cube_msg = make_target_message(self.cube_location, PoseStamped())
                    self.aruco_pose_pub.publish(cube_msg)

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
                    self.play_motion_ac.cancel_goal()
                    self.state = -1
                    print('Service call to pick_up_srv failed: %s' %e)

        
            # State 2:  Move the robot to the other desk.
            if self.state == 2:
                # Rotate the robot
                move_msg = Twist()
                move_msg.angular.z = -1
                move_msg.linear.x = 0

                rate = rospy.Rate(10)
                converged = False
                cnt = 0
                rospy.loginfo("%s: Rotate robot", self.node_name)
                while not rospy.is_shutdown() and cnt < 30:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                rospy.sleep(3)

                # Stop the robot

                move_msg = Twist()
                self.cmd_vel_pub.publish(move_msg)

                # Drive the robot forwards

                move_msg.linear.x = 1
                move_msg.angular.z = 0
                cnt = 0
                while not rospy.is_shutdown() and cnt < 10:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                self.state = 3

                rospy.sleep(3)

                # Stop the robot

                move_msg = Twist()
                self.cmd_vel_pub.publish(move_msg)
            
            # State 3: Calling the place service after publishing the cube location to pick up the cube.
            if self.state == 3:

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
        
            '''
            # Attempt to get placement pose
            if self.state == 2:

                place_pose = rospy.wait_for_message(self.place_pose_top, PoseStamped, 5) # Successfully get position for placing cube.


                print(place_pose) # place_pose is given in the map frame, meaning the robot is travelling to the completely wrong place.
                # How to convert this to the robot frame given that no transform is defined for us?
                # Odom might be the map frame / origo
                goal = MoveBaseGoal()

                goal.target_pose = place_pose
                self.move_base_ac.send_goal(goal)
                success_navigation = self.move_base_ac.wait_for_result(rospy.Duration(60.0))

                if success_navigation:
                    rospy.loginfo('%s: Placement position reached!', self.node_name)
                    self.state = -1

                else:
                    self.move_base_ac.cancel_goal()
                    self.state = -2

                rospy.sleep(3)


            '''
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
    # Since the target_location is a string, we need to split it into doubles or whatever.

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



# import py_trees as pt, py_trees_ros as ptr

# class BehaviourTree(ptr.trees.BehaviourTree):

# 	def __init__(self):

# 		rospy.loginfo("Initialising behaviour tree")

# 		# go to door until at door
# 		b0 = pt.composites.Selector(
# 			name="Go to door fallback", 
# 			children=[Counter(30, "At door?"), Go("Go to door!", 1, 0)]
# 		)

# 		# tuck the arm
# 		b1 = TuckArm()

# 		# go to table
# 		b2 = pt.composites.Selector(
# 			name="Go to table fallback",
# 			children=[Counter(5, "At table?"), Go("Go to table!", 0, -1)]
# 		)

# 		# move to chair
# 		b3 = pt.composites.Selector(
# 			name="Go to chair fallback",
# 			children=[Counter(13, "At chair?"), Go("Go to chair!", 1, 0)]
# 		)

# 		# lower head
# 		b4 = LowerHead()

# 		# become the tree
# 		tree = pt.composites.Sequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
# 		super(BehaviourTree, self).__init__(tree)

# 		# execute the behaviour tree
# 		self.setup(timeout=10000)
# 		while not rospy.is_shutdown(): self.tick_tock(1)


# class Counter(pt.behaviour.Behaviour):

# 	def __init__(self, n, name):

# 		# counter
# 		self.i = 0
# 		self.n = n

# 		# become a behaviour
# 		super(Counter, self).__init__(name)

# 	def update(self):

# 		# count until n
# 		while self.i <= self.n:

# 			# increment count
# 			self.i += 1

# 			# return failure :(
# 			return pt.common.Status.FAILURE

# 		# succeed after counter done :)
# 		return pt.common.Status.SUCCESS


# class Go(pt.behaviour.Behaviour):

# 	def __init__(self, name, linear, angular):

# 		# action space
# 		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
# 		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

# 		# command
# 		self.move_msg = Twist()
# 		self.move_msg.linear.x = linear
# 		self.move_msg.angular.z = angular

# 		# become a behaviour
# 		super(Go, self).__init__(name)

# 	def update(self):

# 		# send the message
# 		rate = rospy.Rate(10)
# 		self.cmd_vel_pub.publish(self.move_msg)
# 		rate.sleep()

# 		# tell the tree that you're running
# 		return pt.common.Status.RUNNING


# class TuckArm(pt.behaviour.Behaviour):

# 	def __init__(self):

# 		# Set up action client
# 		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

# 		# personal goal setting
# 		self.goal = PlayMotionGoal()
# 		self.goal.motion_name = 'home'
# 		self.goal.skip_planning = True

# 		# execution checker
# 		self.sent_goal = False
# 		self.finished = False

# 		# become a behaviour
# 		super(TuckArm, self).__init__("Tuck arm!")

# 	def update(self):

# 		# already tucked the arm
# 		if self.finished: 
# 			return pt.common.Status.SUCCESS
		
# 		# command to tuck arm if haven't already
# 		elif not self.sent_goal:

# 			# send the goal
# 			self.play_motion_ac.send_goal(self.goal)
# 			self.sent_goal = True

# 			# tell the tree you're running
# 			return pt.common.Status.RUNNING

# 		# if I was succesful! :)))))))))
# 		elif self.play_motion_ac.get_result():

# 			# than I'm finished!
# 			self.finished = True
# 			return pt.common.Status.SUCCESS

# 		# if I'm still trying :|
# 		else:
# 			return pt.common.Status.RUNNING
		


# class LowerHead(pt.behaviour.Behaviour):

# 	def __init__(self):

# 		# server
# 		mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
# 		self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
# 		rospy.wait_for_service(mv_head_srv_nm, timeout=30)

# 		# execution checker
# 		self.tried = False
# 		self.tucked = False

# 		# become a behaviour
# 		super(LowerHead, self).__init__("Lower head!")

# 	def update(self):

# 		# try to tuck head if haven't already
# 		if not self.tried:

# 			# command
# 			self.move_head_req = self.move_head_srv("down")
# 			self.tried = True

# 			# tell the tree you're running
# 			return pt.common.Status.RUNNING

# 		# react to outcome
# 		else: return pt.common.Status.SUCCESS if self.move_head_req.success else pt.common.Status.FAILURE


	

if __name__ == "__main__":

	rospy.init_node('main_state_machine')
	try:
		#StateMachine()
		StateMachine()
	except rospy.ROSInterruptException:
		pass


	rospy.spin()