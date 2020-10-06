#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence
# Imports we added
import actionlib
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import JointState
import sys
import warnings
from std_srvs.srv import Empty, SetBool, SetBoolRequest  

from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

global_cube_pose = None
global_cmd_vel_pub = None
global_play_motion_ac = None
global_cube_reset_msg = None
global_cube_reset = None
head_pose = "down"
is_kidnapped = False
is_kidnapped_list = [False] * 5 # One check for every node that might be reset. Localization (0), navigate_pick (1), detect cube (2), navigate_place (3)


# Figure out some way to continually check if cube is detected?
class detect_cube(pt.behaviour.Behaviour):

	"""
	Returns whether the cube is already detected or not
	"""

	def __init__(self, name, aruco_pose_top):

		self.aruco_pose_top = aruco_pose_top
		# Execution checker

		self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
		

		self.found = False

		rospy.loginfo("Initialising cube detection check behaviour.")

		super(detect_cube, self).__init__(name)
	def reset_vars(self):
		global global_cube_pose
		global head_pose
		global is_kidnapped_list

		print("Checking for if kidnapped during cube detection:")
		print(is_kidnapped_list)

		global_cube_pose = None
		self.found = False
		head_pose = "up"
		#is_kidnapped_list[2] = False

	def update(self):
		global is_kidnapped
		global global_cube_pose
		global head_pose
		global is_kidnapped_list

		# send the message
		#rate = rospy.Rate(10)

		if is_kidnapped_list[2]:
			self.reset_vars() # Doesn't set is_kidnapped_list[2] to false so we can reset pick_up, too
		
		if self.found and not is_kidnapped_list[2]:
			return pt.common.Status.SUCCESS

		try:
			if head_pose == "up":
				self.move_head_srv("down")
				rospy.sleep(5)
				head_pose = "down"
			global_cube_pose = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, 2)
		
		except:
			global_cube_pose = None
			self.found = False
			#is_kidnapped = True
			is_kidnapped_list[0], is_kidnapped_list[1], is_kidnapped_list[2] = True, True, True

		if not global_cube_pose:
			print('Looking for cube.')
			return pt.common.Status.RUNNING

		else:
			print('Cube detected.')
			self.found = True
			return pt.common.Status.SUCCESS

class final_check(pt.behaviour.Behaviour):

	"""
	Returns whether the cube is already detected or not
	"""

	def __init__(self, name, aruco_pose_top):

		self.aruco_pose_top = aruco_pose_top

		self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)

		rospy.loginfo("Initialising cube detection check behaviour.")

		super(final_check, self).__init__(name)

	def reset_vars(self):
		global global_cube_pose
		global head_pose
		global is_kidnapped_list

		print("Checking for if kidnapped during final cube detection:")
		print(is_kidnapped_list)

		global_cube_pose = None
		head_pose = "up"

	def update(self):
		global global_cube_pose
		global head_pose
		global is_kidnapped_list

		try:
			#if head_pose == "up":
			self.move_head_srv("down")
			rospy.sleep(5)
			head_pose = "down"
			global_cube_pose = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, 2)
		
		except:
			global_cube_pose = None
			#is_kidnapped = True
			is_kidnapped_list[1], is_kidnapped_list[2], is_kidnapped_list[3] = True, True, True
			is_kidnapped_list[4] = True # For placing the cube.

		if not global_cube_pose:
			print('Cube was not detected on placement area. Returning to pick-up position.')
			# Reset cube location?
			self.move_head_srv("up")
			rospy.sleep(5)
			head_pose = "up"

			# reset_vars?

			move_msg = Twist()
			move_msg.angular.z = 1

			counter = 0
			while counter < 30:
				global_cmd_vel_pub.publish(move_msg)
				counter += 1

			print("Tucking arm...")
			goal = PlayMotionGoal()
			goal.motion_name = 'home'
			goal.skip_planning = True
			global_play_motion_ac.send_goal(goal)
			#global_play_motion_ac.wait_for_result(rospy.Duration(0.0)) # Default 20.0
			try:
				print("Attempting to reset cube position...")
				global_cube_reset(global_cube_reset_msg)
			except:
				print("Cube was not able to be reset!")
			
			return pt.common.Status.RUNNING

		else:
			print('Cube is detected on final table. Mission complete.')
			sys.exit(0)
			return pt.common.Status.SUCCESS

class localise_self(pt.behaviour.Behaviour):
	# the localization step

	def reset_vars(self):
		self.localized = False
		self.move_msg = Twist()
		self.move_msg.angular.z = -1
		self.counter = 0
		self.localization_srv()

	def __init__(self, name, loc_srv, clear_cmaps_srv):

		rospy.loginfo("Initialising localization behaviour.")

		self.localized = False
		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
		self.move_msg = Twist()
		self.move_msg.angular.z = -1
		self.counter = 0
		self.max_count = 60
		self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
		#self.localization_srv = rospy.ServiceProxy(loc_srv_nm, Empty)
		self.localization_srv = loc_srv
		self.localization_srv()

		super(localise_self, self).__init__(name)

	def update(self):
		global is_kidnapped
		global head_pose
		global is_kidnapped_list

		#print('Localizing')

		# if already localized, then publish true
		if self.localized and not is_kidnapped_list[0]:
			return pt.common.Status.SUCCESS

		if is_kidnapped_list[0]:
			self.reset_vars()
			self.localized = False # Seems to work anyway?

			print("Checking for if kidnapped during localisation:")
			print(is_kidnapped_list)

			is_kidnapped_list[0] = False
			#self.counter = 0
			

		if head_pose == "down":
			move_head_req = self.move_head_srv("up")
			rospy.sleep(5)
			head_pose = "up"

		#  otherwise, spin
		rate = rospy.Rate(10)
		self.cmd_vel_pub.publish(self.move_msg)
		rate.sleep()

		# while not fully spun increase counter.
		if self.counter < self.max_count:
			self.counter += 1
			return pt.common.Status.RUNNING

		else:
			# publish empty twist (just in case) to stop
			self.move_msg = Twist()
			self.cmd_vel_pub.publish(self.move_msg)

			particle_cloud = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, 5)
			
			#if any element in the particle cloud is 0.03 or smaller, we say that it has converged
			if particle_cloud_is_converged(particle_cloud):
				self.localized = True
				is_kidnapped_list[0] = False
				return pt.common.Status.SUCCESS
			else:
				print("Warning: AMCL did not converge!")
				#self.localized = False
				self.reset_vars()
				return pt.common.Status.FAILURE

def particle_cloud_is_converged(particle_cloud):
	#if any element in the particle cloud is 0.03 or smaller, we say that it has converged

	max_element = 0

	for element in particle_cloud.pose.covariance:

		if abs(element) > abs(max_element):
			max_element = element
			
	if abs(max_element) < 0.05: # 0.1 seems reasonable so far
		#print(max_element)
		return True 
	return False

class navigation(pt.behaviour.Behaviour):

	def activation_callback(self):
		global head_pose
		print("The navigation action client has been activated.")
		self.move_head_srv("up")
		head_pose = "up"

	def kidnapped_callback(self, feedback): # Cancel goals if kidnapped?
		global is_kidnapped
		global is_kidnapped_list
		#rospy.loginfo("I got feedback")
		particle_cloud = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, 5)
		if not particle_cloud_is_converged(particle_cloud):
			print("*******************K I D N A P P E D*******************")
			# Issue here with returning a success if we cancel all goals. Annoying!
			self.finished_navigation = False
			self.success_navigation = False
			print(self.success_navigation)
			if self.goal_string == "pick":
				is_kidnapped_list[0], is_kidnapped_list[1] = True, True
			elif self.goal_string == "place":
				#is_kidnapped_list[0], is_kidnapped_list[1], is_kidnapped_list[2], is_kidnapped_list[3] = True, True, True, True
				is_kidnapped_list[3] = True
				is_kidnapped_list[0] = True # If kidnapped, robot needs to localise itself again.

			self.move_base_ac.cancel_all_goals()
			# Default is that it is commented in to cancel all goals when this callback is activated.

	def goal_callback(self, state, result):
		rospy.loginfo("I got a result")
		# Should have one for preempted, too

		# This seems to be a contributing factor for why the navigation client usually results in oscillating around end point.
		# When we just returned true this was less likely, I think.
		#"""

		print("State: ")
		print(state)


		if (actionlib.TerminalState.ABORTED or actionlib.TerminalState.PREEMPTED) == state: #or actionlib.TerminalState.PREEMPTED) == state: # Hopefully this should return a false if the action server can't call it a success.
			print("Goal aborted!")
			
			self.success_navigation = False
			"""
			if self.goal_string == "pick":

				# Set a message backing up slightly

				print("Backing up slightly")
				
				back_msg = Twist()
				back_msg.linear.x = -1

				for i in range(10):
					global_cmd_vel_pub.publish(back_msg)

				is_kidnapped_list[0], is_kidnapped_list[1], is_kidnapped_list[2] = True, True, True
			elif self.goal_string == "place":

				# Set a message backing up slightly?
				is_kidnapped_list[0] = True
				is_kidnapped_list[3] = True

			#self.move_base_ac.cancel_all_goals()
			"""

			is_kidnapped_list[0], is_kidnapped_list[1], is_kidnapped_list[2], is_kidnapped_list[3] = True, True, True, True

			try:
				print("Attempting to reset cube position...")
				global_cube_reset(global_cube_reset_msg)
				
			except:
				print("Cube was not able to be reset!")

			print("Tucking arm...")
			goal = PlayMotionGoal()
			goal.motion_name = "home"
			goal.skip_planning = True
			global_play_motion_ac.send_goal(goal)

			self.move_base_ac.cancel_all_goals()
		else:
			self.success_navigation = True
			self.move_base_ac.cancel_all_goals()
		#"""
		#self.success_navigation = True

	def __init__(self, name, goal_string, move_base_ac):
		self.finished_navigation = False
		self.goal = None
		self.pick_pose_top = None
		self.place_pose_top = None
		self.move_base_ac = move_base_ac
		self.goal_string = goal_string
		self.goal_msg = MoveBaseGoal()
		self.success_navigation = None

		self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
		
		# Might be necessary to actually define a MoveBaseGoal message explicitly.
		
		if goal_string == 'pick':
			rospy.loginfo("Initialising pick behaviour.")
			self.pick_pose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
			self.goal = rospy.wait_for_message(self.pick_pose_top, PoseStamped, 5)
			self.goal_msg.target_pose = self.goal

		elif goal_string == 'place':
			rospy.loginfo("Initialising place behaviour.")
			self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')
			self.goal = rospy.wait_for_message(self.place_pose_top, PoseStamped, 5)
			self.goal_msg.target_pose = self.goal

		else:
			print("That's not a valid goal.")

		super(navigation, self).__init__(name)


	def reset_vars(self):
		self.finished_navigation = False
		self.success_navigation = None

		if self.goal_string == "pick":
			self.goal = rospy.wait_for_message(self.pick_pose_top, PoseStamped, 5)

		else:
			self.goal = rospy.wait_for_message(self.place_pose_top, PoseStamped, 5)
		
		self.goal_msg.target_pose = self.goal

	def update(self):
		global is_kidnapped
		global is_kidnapped_list
		
		if is_kidnapped_list[1] and self.goal_string == "pick":

			if not is_kidnapped_list[3]:
				print("Tucking arm...")
				tuckarm() # This just initialises a behaviour. Doesn't actually use it.

			print("Checking for if kidnapped during pick:")
			print(is_kidnapped_list)
			self.reset_vars()
			is_kidnapped_list[1] = False

		elif is_kidnapped_list[3] and self.goal_string == "place":

			print("Checking for if kidnapped during place:")
			print(is_kidnapped_list)
			self.reset_vars()
			is_kidnapped_list[3] = False

		"""
		if self.finished_navigation and not is_kidnapped_list[1] and not is_kidnapped_list[3]:
			return pt.common.Status.SUCCESS
		"""

		if not is_kidnapped_list[1] and (self.goal_string == "pick" and self.finished_navigation):
			return pt.common.Status.SUCCESS

		if not is_kidnapped_list[3] and (self.goal_string == "place" and self.finished_navigation):
			return pt.common.Status.SUCCESS

		try:

			# Try adding a feedback callback to below action client call. Callback will check if kidnapped and cry about it.
			"""
			self.move_base_ac.send_goal(self.goal_msg)
			success_navigation = self.move_base_ac.wait_for_result(rospy.Duration(120.0))
			"""

			if self.goal_string == "pick":
				self.goal = rospy.wait_for_message(self.pick_pose_top, PoseStamped, 5)

			elif self.goal_string == "place":
				self.goal = rospy.wait_for_message(self.place_pose_top, PoseStamped, 5)

			self.goal_msg.target_pose = self.goal

			self.move_base_ac.send_goal(self.goal_msg, active_cb=self.activation_callback, feedback_cb=self.kidnapped_callback, done_cb=self.goal_callback)
			#self.success_navigation = self.move_base_ac.wait_for_result(rospy.Duration(120.0))
			self.move_base_ac.wait_for_result(rospy.Duration(120.0))
			print('Success navigation:')
			print(self.success_navigation)

			if self.success_navigation:
				self.finished_navigation = True
				print("Navigation success!\n We have reached the "+self.goal_string+" pose!")
				return pt.common.Status.SUCCESS

			elif self.success_navigation == False:
				self.move_base_ac.cancel_goal()
				print("I am lost. Lost beyond words.")
				return pt.common.Status.FAILURE

			else:
				return pt.common.Status.RUNNING

		except:
			print('Oh no')

		

		"""
		if self.success_navigation:
			self.finished_navigation = True
			print("Navigation success!\n We have reached the "+self.goal_string+" pose!")
			return pt.common.Status.SUCCESS

		elif self.success_navigation == False:
			self.move_base_ac.cancel_goal()
			print("I am lost. Lost beyond words.")
			return pt.common.Status.FAILURE

		else:
			return pt.common.Status.RUNNING"""

		# meme
		
class pick_up(pt.behaviour.Behaviour):
	
	def __init__(self, name, pick_srv_nm, aruco_pose_pub):
		
		self.aruco_pose_pub = aruco_pose_pub
		self.pick_up_srv = rospy.ServiceProxy(pick_srv_nm, SetBool)
		self.picked_up = False

		rospy.loginfo("Initialising pick up behaviour.")		

		super(pick_up, self).__init__(name)

	def update(self):

		global is_kidnapped_list

		if is_kidnapped_list[2]: # If the robot has to navigate back to pick... originally [1]. Why?
			print("Checking for if kidnapped during pick_up action itself:")
			print(is_kidnapped_list)
			is_kidnapped_list[2] = False
			self.picked_up = False 

		if self.picked_up:
			return pt.common.Status.SUCCESS

		self.aruco_pose_pub.publish(global_cube_pose)
		self.pick_up_request = self.pick_up_srv()


		if self.pick_up_request.success:

			print('Grabbing the cube succeeded!')
			self.picked_up = True
			return pt.common.Status.SUCCESS
		
		elif not self.pick_up_request.success:
			print('Grabbing the cube failed!')
			# Make backup?
			return pt.common.Status.FAILURE

class place(pt.behaviour.Behaviour):
	
	def __init__(self, name, place_srv_nm, aruco_pose_pub):
		self.aruco_pose_pub = aruco_pose_pub
		self.place_srv = rospy.ServiceProxy(place_srv_nm, SetBool)
		self.placed = False

		rospy.loginfo("Initialising placement behaviour.")

		super(place, self).__init__(name)

	def update(self):
		global is_kidnapped_list

		if is_kidnapped_list[4]: # Originally [3]. If robot previously failed to place cube.
			self.placed = False 

		if self.placed:
			return pt.common.Status.SUCCESS

		self.aruco_pose_pub.publish(global_cube_pose)
		self.place_request = self.place_srv()

		if self.place_request.success:
			print("Placing the cube succeeded!")
			self.placed = True
			return pt.common.Status.SUCCESS
		
		elif not self.place_request.success:
			print("Placing the cube failed!")

			move_msg = Twist()
			move_msg.angular.z = 1

			counter = 0
			while counter < 30:
				global_cmd_vel_pub.publish(move_msg)
				counter += 1


			print("Tucking arm...")
			goal = PlayMotionGoal()
			goal.motion_name = "home"
			goal.skip_planning = True
			global_play_motion_ac.send_goal(goal)
			#global_play_motion_ac.wait_for_result(rospy.Duration(0.0)) # default 20.

			try:
				print("Attempting to reset cube position...")
				global_cube_reset(global_cube_reset_msg)
			except:
				print("Cube was not able to be reset!")

			is_kidnapped_list[1] = True # navigate to pick
			is_kidnapped_list[2] = True # detect the cube
			is_kidnapped_list[3] = True # Navigate to place



			return pt.common.Status.FAILURE
		"""
		else:
			return pt.common.Status.RUNNING
		"""

class A_level_BehaviourTree(ptr.trees.BehaviourTree):	

	"""
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
	"""
	
	def __init__(self):

		global global_cmd_vel_pub
		global global_play_motion_ac
		global global_cube_reset_msg
		global global_cube_reset

		rospy.loginfo("Initialising behaviour tree")

		# Access ROS parameters:

		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
		self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.localization_srv_nm = rospy.get_param(rospy.get_name() + '/global_loc_srv')
		self.clear_cmaps_srv_nm = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
		self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
		self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
		self.pick_pose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
		self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
		self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')
		self.amcl_estimate_top = rospy.get_param(rospy.get_name() + '/amcl_estimate')

		self.localization_srv = rospy.ServiceProxy(self.localization_srv_nm, Empty)
		self.clear_costmaps_srv = rospy.ServiceProxy(self.clear_cmaps_srv_nm, Empty)

		global_cube_reset_msg = ModelState()

		### - Reset pose for the cube, to be used later - ###

		global_cube_reset_msg.model_name = 'aruco_cube'
		global_cube_reset_msg.pose.position.x = -1.130530
		global_cube_reset_msg.pose.position.y = -6.653650
		global_cube_reset_msg.pose.position.z = 0.86250
		global_cube_reset_msg.pose.orientation.x = 0
		global_cube_reset_msg.pose.orientation.y = 0 # roll
		global_cube_reset_msg.pose.orientation.z = 0
		global_cube_reset_msg.pose.orientation.w = 1 # pitch

		### - Reset pose for the cube, to be used later - ###
		
		# Wait for service providers

		### - Reset Cube position service - ###

		rospy.wait_for_service('/gazebo/set_model_state')
		global_cube_reset = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

		### - Reset Cube position service - ###

		rospy.wait_for_service(self.mv_head_srv_nm, timeout = 30)
		rospy.wait_for_service(self.pick_srv_nm, timeout = 30)
		rospy.wait_for_service(self.place_srv_nm, timeout = 30)
		rospy.wait_for_service(self.localization_srv_nm, timeout = 30)
		rospy.wait_for_service(self.clear_cmaps_srv_nm, timeout = 30)

		# Test of placing the requests for localisation and clear cost maps

		self.localization_req = self.localization_srv()
		self.clear_costmaps_req = self.clear_costmaps_srv()

		# Instantiate publishers

		global_cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

		self.aruco_pose_pub = rospy.Publisher(self.aruco_pose_top, PoseStamped, queue_size=10)

		# Set up action clients
		rospy.loginfo("%s: Waiting for play_motion action server...")
		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
		if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
			rospy.logerr("%s: Could not connect to /play_motion action server")
			exit()

		global_play_motion_ac = self.play_motion_ac
		
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
		branch_1 = localise_self("localization", self.localization_srv, self.clear_costmaps_srv)

		branch_2 = navigation("pick navigation", "pick", self.move_base_ac)

		# to detect cube, move head down and then use aruco detection
		branch_3 = detect_cube("detect cube", self.aruco_pose_top)

		# pick up the cube
		branch_4 = pick_up("pick_up", self.pick_srv_nm, self.aruco_pose_pub) # We should not have to publish cube pose again! Dum dum

		# redoing branch_1 with a new destination (place pose)

		branch_5 = movehead("up")

		branch_6 = navigation("place navigation", "place", self.move_base_ac)

		# putting down the cube in the same relative pose as we picked it up with
		# of course at the 'place pose' instead of the 'pick pose'
		branch_7 = place("place", self.place_srv_nm, self.aruco_pose_pub) # Move head down?

		branch_8 = final_check("final check", self.aruco_pose_top)

		# become the tree
		tree = RSequence(name="Main sequence", children=[branch_0, branch_1, branch_2, branch_3, branch_4, branch_5, branch_6, branch_7, branch_8])
		super(A_level_BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)
		



if __name__ == "__main__":

	warnings.filterwarnings("ignore")

	rospy.init_node('main_behaviour_tree')
	try:
		#BehaviourTree()
		A_level_BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
