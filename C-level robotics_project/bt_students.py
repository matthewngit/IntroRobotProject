#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence
# Imports we added
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import JointState
import sys
from std_srvs.srv import Empty, SetBool, SetBoolRequest  

global_cube_pose = None

'''

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

		# go to door until at door
		b0 = pt.composites.Selector(
			name="Go to door fallback", 
			children=[counter(30, "At door?"), go("Go to door!", 1, 0)]
		)

		# tuck the arm
		b1 = tuckarm()

		# go to table
		b2 = pt.composites.Selector(
			name="Go to table fallback",
			children=[counter(5, "At table?"), go("Go to table!", 0, -1)]
		)

		# move to chair
		b3 = pt.composites.Selector(
			name="Go to chair fallback",
			children=[counter(13, "At chair?"), go("Go to chair!", 1, 0)]
		)

		# lower head
		b4 = movehead("down")

		# become the tree
		tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)

'''	
# Figure out some way to continually check if cube is detected?
class detect_cube(pt.behaviour.Behaviour):

	'''
	Returns whether the cube is already detected or not
	'''


	def __init__(self, name, aruco_pose_top):

		self.aruco_pose_top = aruco_pose_top
		# Execution checker

		self.found = False

		rospy.loginfo("Initialising cube detection check behaviour.")

		super(detect_cube, self).__init__(name)

	def update(self):

		global global_cube_pose

		# send the message
		#rate = rospy.Rate(10)
		
		if self.found:
			return pt.common.Status.SUCCESS

		cube_pose = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, 5)

		if not cube_pose:
			print('Looking for cube.')
			return pt.common.Status.RUNNING

		else:
			print('Cube detected.')
			self.found = True
			global_cube_pose = cube_pose
			return pt.common.Status.SUCCESS

		#self.cmd_vel_pub.publish(self.move_msg)

		#rate.sleep()

		# tell the tree that you're running

class pick_up(pt.behaviour.Behaviour):
	
	def __init__(self, name, pick_srv_nm, aruco_pose_pub):
		
		
		self.aruco_pose_pub = aruco_pose_pub
		self.pick_up_srv = rospy.ServiceProxy(pick_srv_nm, SetBool)
		self.picked_up = False

		rospy.loginfo("Initialising pick up behaviour.")		

		super(pick_up, self).__init__(name)

	def update(self):

		if self.picked_up:
			return pt.common.Status.SUCCESS

		self.aruco_pose_pub.publish(global_cube_pose)
		self.pick_up_request = self.pick_up_srv()

		#print(global_cube_pose)

		if self.picked_up:
			return pt.common.Status.SUCCESS

		if self.pick_up_request.success:

			print('Cube the cube succeeded!')
			self.picked_up = True
			return pt.common.Status.SUCCESS
		
		elif not self.pick_up_request.success:
			print('Grabbing the cube failed!')
			return pt.common.Status.FAILURE
		'''
		else:
			return pt.common.Status.RUNNING
		'''

class place(pt.behaviour.Behaviour):
	
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

		if self.placed:
			self.placed = True

		if self.place_request.success:
			print('Placing the cube succeeded!')
			self.placed = True
			return pt.common.Status.SUCCESS
		
		elif not self.place_request.success:
			print('Placing the cube failed!')
			return pt.common.Status.FAILURE
		'''
		else:
			return pt.common.Status.RUNNING
		'''

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

		# Wait for service providers

		self.cube_detector = detect_cube('cube detection', self.aruco_pose_top)

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
		branch_0 = tuckarm()

		branch_1 = pt.composites.Sequence(
			name = 'cube detection',
			children = [movehead('down'), self.cube_detector]
		)
		
		branch_2 = pick_up('pick_up', self.pick_srv_nm, self.aruco_pose_pub)

		branch_3 = movehead('up')

		branch_4 = pt.composites.Selector(
			name = 'rotate',
			children = [counter(29, 'rotation duration reached?'), go('Rotate!', 0, -1)]
		)

		branch_5 = pt.composites.Selector(
			name = 'stop',
			children = [counter(1, 'Is stopped?'), go('Stop!', 0, 0)]
		)


		branch_6 = pt.composites.Selector(
			name = 'translate',
			children = [counter(9, 'translation duration reached?'), go('Translate!', 1, 0)]
		)

		branch_7 = place('place', self.place_srv_nm, self.aruco_pose_pub)

		# become the tree
		tree = RSequence(name="Main sequence", children=[branch_0, branch_1, branch_2, branch_3, branch_4, branch_5, branch_6, branch_7])
		super(C_level_BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)
		



if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		#BehaviourTree()
		C_level_BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
