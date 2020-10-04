#!/usr/bin/env python

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

global_cube_pose = None


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

		# send the message, if found already
		if self.found:
			return pt.common.Status.SUCCESS
		
		# Get cube pose (detect the cube)
		cube_pose = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, 5)

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

		rospy.loginfo("Initialising pick up behaviour.")		

		super(pick_up, self).__init__(name)

	def update(self):
		
		# same as before, if cube is picked up, always return true
		if self.picked_up:
			return pt.common.Status.SUCCESS

		# If not already picked up, do the following:

		# publish the global pose from before (in detect cube)
		self.aruco_pose_pub.publish(global_cube_pose)
		self.pick_up_request = self.pick_up_srv()

		# if success, (return true in the beginning)
		if self.pick_up_request.success:
			print('Cube the cube succeeded!')
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

		rospy.loginfo("Initialising placement behaviour.")

		super(place, self).__init__(name)

	def update(self):

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
			return pt.common.Status.FAILURE


class C_level_BehaviourTree(ptr.trees.BehaviourTree):
	
	def __init__(self):

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

		# Instantiate publishers
		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
		self.aruco_pose_pub = rospy.Publisher(self.aruco_pose_top, PoseStamped, queue_size=10)

		# Set up action clients
		rospy.loginfo("%s: Waiting for play_motion action server...")
		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
		if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
			rospy.logerr("%s: Could not connect to /play_motion action server")
			exit()
		rospy.sleep(1)

		# The reset 'home' pose
		branch_0 = tuckarm()

		# Detect cube
		branch_1 = pt.composites.Sequence(
			name = 'cube detection',
			children = [movehead('down'), self.cube_detector]
		)
		
		# Pick up Cube
		branch_2 = pick_up('pick_up', self.pick_srv_nm, self.aruco_pose_pub)

		# Move head up
		branch_3 = movehead('up')

		# Rotate 180 degrees
		branch_4 = pt.composites.Selector(
			name = 'rotate',
			children = [counter(29, 'rotation duration reached?'), go('Rotate!', 0, -1)]
		)

		# Stop spinning
		branch_5 = pt.composites.Selector(
			name = 'stop',
			children = [counter(1, 'Is stopped?'), go('Stop!', 0, 0)]
		)

		# Move to the other table
		branch_6 = pt.composites.Selector(
			name = 'translate',
			children = [counter(9, 'translation duration reached?'), go('Translate!', 1, 0)]
		)

		# Place down cube
		branch_7 = place('place', self.place_srv_nm, self.aruco_pose_pub)

		# become the tree
		tree = RSequence(name="Main sequence", children=[branch_0, branch_1, branch_2, branch_3, branch_4, branch_5, branch_6, branch_7])
		super(C_level_BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)
		

if __name__ == "__main__":
	# The main function as given
	rospy.init_node('main_state_machine')
	try:
		C_level_BehaviourTree()
	except rospy.ROSInterruptException:
		pass
	rospy.spin()
