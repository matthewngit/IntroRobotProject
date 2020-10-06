#!/usr/bin/env python
# -*- coding: utf-8 -*-

# DD2410 - Project Mobile Manipulation
# C-level Behavior Tree
# Matthew Norström 970313, Marcus Jirwe 960903

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence
# Imports we added
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import JointState
import sys
from std_srvs.srv import Empty, SetBool, SetBoolRequest

from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState


global_cube_pose = None
global_cmd_vel_pub = None
global_play_motion_ac = None

global_cube_reset_msg = None
global_cube_reset = None

retry = False
reset_pick = False
reset_place = False
reset_go_to_pick = False
reset_go_to_place = False
reset_final_check = False
reset_detect_cube = False


class detect_cube(pt.behaviour.Behaviour):

	# Returns whether the cube is already detected or not
	def __init__(self, name, aruco_pose_top):

		self.aruco_pose_top = aruco_pose_top
		self.found = False

		rospy.loginfo("Initialising cube detection check behaviour.")

		# According to the style of behavior Trees (in the other file)
		super(detect_cube, self).__init__(name)

	def update(self):
		global global_cube_pose
		global reset_detect_cube

		if reset_detect_cube:
			self.found = False
			reset_detect_cube = False

		# send the message, if found already
		if self.found:
			return pt.common.Status.SUCCESS
		
		# Get cube pose (detect the cube)

		try:
			cube_pose = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, 5)
		except:
			cube_pose = None

		# If the cube was not detected, keep on looking
		if not cube_pose:
			print('Looking for cube.')
			return pt.common.Status.RUNNING

		# If detected, set found=True (send message at start every time)

		else:
			print('Cube detected.')
			self.found = True
			#set the cube_pose
			global_cube_pose = cube_pose
			return pt.common.Status.SUCCESS


class pick_up(pt.behaviour.Behaviour):
	# The pick up class
	def __init__(self, name, pick_srv_nm, aruco_pose_pub):	
		self.aruco_pose_pub = aruco_pose_pub
		self.pick_up_srv = rospy.ServiceProxy(pick_srv_nm, SetBool)
		self.picked_up = False
		self.reset = False

		rospy.loginfo("Initialising pick up behaviour.")		

		super(pick_up, self).__init__(name)

	def update(self):

		global reset_pick

		if reset_pick:
			self.picked_up = False
			reset_pick = False
		
		# same as before, if cube is picked up, always return true
		if self.picked_up:
			return pt.common.Status.SUCCESS

		# If not already picked up, do the following:

		# publish the global pose from before (in detect cube)
		self.aruco_pose_pub.publish(global_cube_pose)
		self.pick_up_request = self.pick_up_srv()

		# if success, (return true in the beginning)
		if self.pick_up_request.success:
			print('Grabbing the cube succeeded!')
			# by setting 'self.picked_up' to true
			self.picked_up = True
			return pt.common.Status.SUCCESS
		
		# Otherwise, keep on truckin'
		elif not self.pick_up_request.success:
			print('Grabbing the cube failed!')
			return pt.common.Status.FAILURE


class place(pt.behaviour.Behaviour):
	# exactly like 'pick_up', but for placing the cube 
	# (we'll bake these functions together in the A-level)
	def __init__(self, name, place_srv_nm, aruco_pose_pub):
		
		self.aruco_pose_pub = aruco_pose_pub
		self.place_srv = rospy.ServiceProxy(place_srv_nm, SetBool)
		self.placed = False
		self.reset = False

		rospy.loginfo("Initialising placement behaviour.")

		super(place, self).__init__(name)

	def update(self):

		global reset_detect_cube
		global reset_go_to_pick
		global reset_go_to_place
		global reset_pick
		global reset_place

		if reset_place:
			self.placed = False
			self.reset_place = False

		if self.placed:
			return pt.common.Status.SUCCESS

		self.aruco_pose_pub.publish(global_cube_pose)
		self.place_request = self.place_srv()

		if self.place_request.success:
			print('Placing the cube succeeded!')
			self.placed = True
			return pt.common.Status.SUCCESS
		
		elif not self.place_request.success:
			print('Placing the cube failed!')
			reset_go_to_pick, reset_detect_cube, reset_pick, reset_go_to_place, reset_place = True, True, True, True, True

			try:
				print("Attempting to reset cube position...")
				global_cube_reset(global_cube_reset_msg)
			except:
				print("Cube was not able to be reset!")
			
			return pt.common.Status.FAILURE

class final_check(pt.behaviour.Behaviour):

	def __init__(self, name, aruco_pose_top):

		self.aruco_pose_top = aruco_pose_top
		self.found = False # Variable for whether the cube is detected at the goal.
		rospy.loginfo("Initialising cube detection check behaviour.")
		
		super(final_check, self).__init__(name)

	def update(self):
		global global_cube_pose
		global reset_detect_cube
		global reset_go_to_pick
		global reset_go_to_place
		global reset_pick
		global reset_place
		global reset_final_check

		if self.found and reset_final_check: # If this is after first try.
			reset_final_check = False

		if self.found:
			return pt.common.Status.SUCCESS

		# Attempt to find cube at the goal.

		try:
			global_cube_pose = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, 5)
		except:
			global_cube_pose = None

		if not global_cube_pose:
			print("Cube not found at placement area. Returning to pick.")
			reset_go_to_pick, reset_detect_cube, reset_pick, reset_go_to_place, reset_place, reset_final_check = True, True, True, True, True, True

			try:
				print("Attempting to reset cube position...")
				global_cube_reset(global_cube_reset_msg)
			except:
				print("Cube was not able to be reset!")


			return pt.common.Status.FAILURE

		else:
			print("Cube detected at placement area. Mission success.")
			self.found = True
			sys.exit(0)
			return pt.common.Status.SUCCESS

class travel(pt.behaviour.Behaviour):

	def __init__(self, name, max_rotate_count, max_translate_count, destination):
		
		self.at_destination = False
		self.max_rotate_count = max_rotate_count
		self.rotate_count = 0
		self.max_translate_count = max_translate_count
		self.translate_count = 0
		self.destination = destination
		self.rotate_msg = Twist()
		self.translate_msg = Twist()
		self.stop_msg = Twist() # Unnecessary?
		self.first_try = False
		self.reset = False
		self.reset_pose = False
		

		if self.destination == "pick": # If our destination is the start we set the "self.first_try" to True.
			self.rotate_msg.angular.z = 1
			self.first_try = True

		else:
			self.rotate_msg.angular.z = -1

		self.translate_msg.linear.x = 1.1

		super(travel, self).__init__(name)

	def update(self):

		global reset_go_to_pick
		global reset_go_to_place

		if reset_go_to_pick and self.destination == "pick":
			self.reset = True

		elif reset_go_to_place and self.destination == "place":
			self.reset = True

		if self.reset:
			self.rotate_count = 0
			self.translate_count = 0
			self.at_destination = False
			self.reset = False

			if self.destination == "pick":
				goal = PlayMotionGoal()
				goal.motion_name = 'home' # Should probably wait for the result since the destination is so close.
				goal.skip_planning = True
				global_play_motion_ac.send_goal(goal)
				global_play_motion_ac.wait_for_result(rospy.Duration(0.0))
				print("Pose reset.")

				reset_go_to_pick = False

			else:
				reset_go_to_place = False

		if self.first_try:
			self.at_destination = True
			self.first_try = False

		if self.at_destination:
			return pt.common.Status.SUCCESS

		rate = rospy.Rate(10)

		while self.rotate_count < self.max_rotate_count:

			global_cmd_vel_pub.publish(self.rotate_msg)
			rate.sleep()
			self.rotate_count += 1

			if self.rotate_count == (self.max_rotate_count):
				global_cmd_vel_pub.publish(self.stop_msg)
				print("Rotation finished.")
				rospy.sleep(3)

			return pt.common.Status.RUNNING

		while self.translate_count < self.max_translate_count:

			global_cmd_vel_pub.publish(self.translate_msg)
			rate.sleep()
			self.translate_count += 1

			if self.translate_count == (self.max_translate_count):
				global_cmd_vel_pub.publish(self.stop_msg)
				print("Translation finished.")
				rospy.sleep(3)

			return pt.common.Status.RUNNING

		#global_cmd_vel_pub.publish(self.stop_msg)
		self.at_destination = True
		self.reset = False

		rospy.sleep(5)


		return pt.common.Status.SUCCESS


class C_level_BehaviourTree(ptr.trees.BehaviourTree):
	
	def __init__(self):

		global global_cmd_vel_pub
		global global_cube_reset_msg
		global global_cube_reset
		global global_play_motion_ac


		# Define parameters for resetting cube

		global_cube_reset_msg = ModelState()

		global_cube_reset_msg.model_name = 'aruco_cube'
		global_cube_reset_msg.pose.position.x = -1.130530
		global_cube_reset_msg.pose.position.y = -6.653650
		global_cube_reset_msg.pose.position.z = 0.86250
		global_cube_reset_msg.pose.orientation.x = 0
		global_cube_reset_msg.pose.orientation.y = 0 # roll
		global_cube_reset_msg.pose.orientation.z = 0
		global_cube_reset_msg.pose.orientation.w = 1 # pitch

		rospy.loginfo("Initialising behaviour tree")

		# Access ROS parameters:
		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
		self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
		self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
		self.pick_pose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
		self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
		self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')

		self.cube_detector = detect_cube('cube detection', self.aruco_pose_top)
		
		# Wait for service providers
		rospy.wait_for_service(self.mv_head_srv_nm, timeout = 30)
		rospy.wait_for_service(self.pick_srv_nm, timeout = 30)
		rospy.wait_for_service(self.place_srv_nm, timeout = 30)

		rospy.wait_for_service('/gazebo/set_model_state')
		global_cube_reset = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

		# Instantiate publishers
		global_cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
		self.aruco_pose_pub = rospy.Publisher(self.aruco_pose_top, PoseStamped, queue_size=10)

		# Set up action clients
		rospy.loginfo("%s: Waiting for play_motion action server...")
		global_play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
		if not global_play_motion_ac.wait_for_server(rospy.Duration(1000)):
			rospy.logerr("%s: Could not connect to /play_motion action server")
			exit()
		rospy.sleep(1)

		# The reset 'home' pose
		branch_0 = tuckarm()

		branch_0_5 = travel("go to pick", 30, 9, "pick")

		# Detect cube
		branch_1 = pt.composites.Sequence(
			name = 'cube detection',
			children = [movehead('down'), self.cube_detector]
		)
		
		# Pick up Cube
		branch_2 = pick_up('pick_up', self.pick_srv_nm, self.aruco_pose_pub)

		# Move head up
		#branch_3 = movehead('up')

		# Rotate 180 degrees
		'''
		branch_4 = pt.composites.Selector(
			name = 'rotate',
			children = [counter(29, 'rotation duration reached?'), go('Rotate!', 0, -1)]
		)
		'''

		branch_4 = travel("go to place", 30, 9, "place")

		# Stop spinning
		'''
		branch_5 = pt.composites.Selector(
			name = 'stop',
			children = [counter(1, 'Is stopped?'), go('Stop!', 0, 0)]
		)
		'''

		# Move to the other table
		'''
		branch_6 = pt.composites.Selector(
			name = 'translate',
			children = [counter(9, 'translation duration reached?'), go('Translate!', 1, 0)]
		)
		'''

		# Place down cube
		branch_5 = place('place', self.place_srv_nm, self.aruco_pose_pub)

		branch_6 = final_check("final check", self.aruco_pose_top)

		# become the tree
		tree = RSequence(name="Main sequence", children=[branch_0, branch_0_5, branch_1, branch_2, branch_4, branch_5, branch_6])
		super(C_level_BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)

		# Maybe needs to not define movehead only as nodes.
		

if __name__ == "__main__":
	# The main function as given
	rospy.init_node('main_state_machine')
	try:
		C_level_BehaviourTree()
	except rospy.ROSInterruptException:
		pass
	rospy.spin()
