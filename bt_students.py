#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence
# Imports we added
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import JointState
import sys
from std_srvs.srv import Empty, SetBool, SetBoolRequest  

global_cube_pose = None


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

class localise_self(pt.behaviour.Behaviour):
	# the localization step

	def __init__(self, name, loc_srv_nm, clear_cmaps_srv_nm):

		rospy.loginfo("Initialising localization behaviour.")

		self.localized = False
		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
		self.move_msg = Twist()
		self.move_msg.angular.z = -1
		self.counter = 0
		self.max_count = 60
		
		self.localization_srv = rospy.ServiceProxy(loc_srv_nm, Empty)
		self.clear_costmaps_srv = rospy.ServiceProxy(clear_cmaps_srv_nm, Empty)

		self.localization_req = self.localization_srv()
		self.clear_costmaps_req = self.clear_costmaps_srv()
		

		super(localise_self, self).__init__(name)

	def update(self):

		# if already localized, then publish true
		if self.localized:
			return pt.common.Status.SUCCESS

		# otherwise, spin
		rate = rospy.Rate(10)
		self.cmd_vel_pub.publish(self.move_msg)
		rate.sleep()

		
		#clear_costmaps_req = self.clear_costmaps_srv()
		# while not fully spun increase counter.
		if self.counter < self.max_count:
			self.counter += 1
			return pt.common.Status.RUNNING

		else:
			# publish empty twist (just in case) to stop
			self.move_msg = Twist()
			self.cmd_vel_pub.publish(self.move_msg)

			#set localized boolean to true
			self.localized = True
			return pt.common.Status.SUCCESS

class navigation(pt.behaviour.Behaviour):

	def __init__(self, name, goal_string, move_base_ac):
		self.finished_navigation = False
		self.goal = None
		self.pick_pose_top = None
		self.place_pose_top = None
		self.move_base_ac = move_base_ac
		self.goal_string = goal_string
		self.goal_msg = MoveBaseGoal()
		
		# Might be necessary to actually define a MoveBaseGoal message explicitly.
		
		if goal_string == 'pick':
			rospy.loginfo("Initialising pick behaviour.")
			self.pick_pose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
			self.goal = rospy.wait_for_message(self.pick_pose_top, PoseStamped, 5)
			print("The pick message: ")
			print(self.goal)
			self.goal_msg.target_pose = self.goal



		elif goal_string == 'place':
			rospy.loginfo("Initialising place behaviour.")
			self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')
			self.goal = rospy.wait_for_message(self.place_pose_top, PoseStamped, 5)
			print("The place message: ")
			print(self.goal)
			self.goal_msg.target_pose = self.goal

		else:
			print("That's not a valid goal.")

		super(navigation, self).__init__(name)

	def update(self):

		if self.finished_navigation:
			return pt.common.Status.SUCCESS

		self.move_base_ac.send_goal(self.goal_msg)
		success_navigation = self.move_base_ac.wait_for_result(rospy.Duration(120.0))

		if success_navigation:
			self.finished_navigation = True
			print("Navigation success!\n We have reached the "+self.goal_string+" pose!")
			return pt.common.Status.SUCCESS

		else:
			self.move_base_ac.cancel_goal()
			print("I am lost. Lost beyond words.")
			return pt.common.Status.FAILURE
		
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

class A_level_BehaviourTree(ptr.trees.BehaviourTree):	

	'''

	Tasks necessary for the A-grade project:

    1. Robot has localized itself in the apartment.
    2. Navigation to picking pose.
    3. Cube detected.
    4. Complete picking task.
    5. Navigation with cube to second table.
    6. Complete placing task.
    7. Cube placed on table?
        Yes: end of mission.
        No: go back to state 2 and repeat until success. Need to respawn cube.

	Obs 1: At any time during the navigation, a bad-intentioned TA might kidnap your robot again. 
	Your behavior tree must be able to detect this and react to it so that the robot always knows its true position. 
	Kidnap the robot yourself during your development to test your solution (the robot can be moved in Gazebo manually).

	Obs 2: The robot uses a particle filter for localization. 
	Use the state of the distribution of the particles to know when the filter has converged. 
	Other solutions will not be accepted.

	'''
	
	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

		# Access ROS parameters:

		#self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
		self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.localization_srv_nm = rospy.get_param(rospy.get_name() + '/global_loc_srv')
		self.clear_cmaps_srv_nm = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
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
		#self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
		self.aruco_pose_pub = rospy.Publisher(self.aruco_pose_top, PoseStamped, queue_size=10)

		# Set up action clients
		rospy.loginfo("%s: Waiting for play_motion action server...")
		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
		if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
			rospy.logerr("%s: Could not connect to /play_motion action server")
			exit()
		
		rospy.loginfo("%s: Waiting for move_base action server...")
		self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
		if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
			rospy.logerr("%s: Could not connect to /move_base action server")
			exit()
		rospy.loginfo("%s: Connected to /move_base action server")
        

		rospy.sleep(1)

		# The "home" reset/initialization pose neccessary for each run
		branch_0 = tuckarm()

		# Probably need to transform the position of pick & place. Also dynamic checks.

		# moving head up and spinning to localize
		# and then activating the navigation
		branch_1 = pt.composites.Sequence(
			name = 'localization and navigation to pick',
			children = [movehead("up"), localise_self("localization", self.localization_srv_nm, self.clear_cmaps_srv_nm), navigation("pick navigation", "pick", self.move_base_ac)]
		)

		# to detect cube, move head down and then use aruco detection
		branch_2 = pt.composites.Sequence(
			name = 'cube detection',
			children = [movehead('down'), detect_cube('Detect cube', self.aruco_pose_top)]
		)

		# pick up the cube
		branch_3 = pick_up('pick_up', self.pick_srv_nm, self.aruco_pose_pub) # We should not have to publish cube pose again! Dum dum

		# redoing branch_1 with a new destination (place pose)
		branch_4 = pt.composites.Sequence(
			name = 'localization and navigation to place',
			children = [movehead("up"), navigation("place navigation", "place", self.move_base_ac)]
		)

		# putting down the cube in the same relative pose as we picked it up with
		# of course at the 'place pose' instead of the 'pick pose'
		branch_5 = place('place', self.place_srv_nm, self.aruco_pose_pub) # Move head down?

		# become the tree
		tree = RSequence(name="Main sequence", children=[branch_0, branch_1, branch_2, branch_3, branch_4, branch_5])
		super(A_level_BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)
		



if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		#BehaviourTree()
		A_level_BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
