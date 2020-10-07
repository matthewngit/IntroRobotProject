#!/usr/bin/env python
# -*- coding: utf-8 -*-

# DD2410 - Project Mobile Manipulation
# A-level Behavior Tree
# Matthew Norström 970313, Marcus Jirwe 960903

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence
# Imports we added
import actionlib
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
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
head_pose = "down"
# One check for every node that might be reset. Localization (0), navigate_pick (1), detect cube (2), navigate_place (3)
reset_list = [False] * 5
# list[0] = re-localize
# list[1] = re-navigate to "pick"
# list[2] = re-check for cube and re-pick the cube
# list[3] = re-navigate to "place"
# list[4] = re-do the placement of the cube


# Figure out some way to continually check if cube is detected?
class detect_cube(pt.behaviour.Behaviour):
	# Returns whether the cube is already detected or not

	def __init__(self, name, aruco_pose_top):
		self.aruco_pose_top = aruco_pose_top
		self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
		self.found = False

		super(detect_cube, self).__init__(name)

	def reset_vars(self):
		global global_cube_pose
		global head_pose
		global reset_list

		# to redo the check for cube and pick, we must reset the vars
		global_cube_pose = None
		self.found = False
		head_pose = "up"

	def update(self):
		global is_kidnapped
		global global_cube_pose
		global head_pose
		global reset_list

		if reset_list[2]: 
			# if we have to redo check and pick for cube
			self.reset_vars() 
		
		if self.found and not reset_list[2]:
			# always return true until we're kidnapped, if we have picked up the cube
			return pt.common.Status.SUCCESS

		try:
			if head_pose == "up":
				# move head down to see cube, if we're looking up
				self.move_head_srv("down")
				rospy.sleep(5)
				head_pose = "down"
			global_cube_pose = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, 2)
		
		except:
			# if the cube isn't there, then we need to redo localization, navigation, and cube check/pick
			global_cube_pose = None
			self.found = False
			reset_list[0], reset_list[1], reset_list[2] = True, True, True

		if not global_cube_pose:
			# continue to look, if we haven't found it yet
			return pt.common.Status.RUNNING

		else:
			# when we've found it, return success and set found=true (so that we don't keep on looking)
			self.found = True
			return pt.common.Status.SUCCESS

class final_check(pt.behaviour.Behaviour):
	# Returns whether the cube is already detected or not

	def __init__(self, name, aruco_pose_top):
		self.aruco_pose_top = aruco_pose_top
		self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)

		super(final_check, self).__init__(name)

	def update(self):
		global global_cube_pose
		global head_pose
		global reset_list

		print("Performing a final check for cube at goal area.")

		try:
			# move head down to see cube
			self.move_head_srv("down")
			rospy.sleep(5)
			head_pose = "down"
			# look for cube
			global_cube_pose = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, 2)
		
		except:
			# if we didn't find the cube
			global_cube_pose = None
			reset_list[1], reset_list[2], reset_list[3], reset_list[4] = True, True, True, True

		if not global_cube_pose:
			print("Cube not detected at placement area. Returning to try again.")
			# look up, so we can navigate back
			self.move_head_srv("up")
			rospy.sleep(5)
			head_pose = "up"

			# reset arm pose, so that we won't crash into things with arm
			tuckarm_during_run()

			try:
				# respawn the cube, in case it has fallen or disappeared.
				global_cube_reset(global_cube_reset_msg)
			except:
				print("Cube was not able to be reset!")
			
			return pt.common.Status.RUNNING

		else:
			# Done, we're finally done with this hell!
			print('Cube is detected on final table. Mission complete.')
			sys.exit(0)
			return pt.common.Status.SUCCESS

def tuckarm_during_run():
	# a function that activates the "tuckarm" upper motion
	# can be called upon during lower movement.
	goal = PlayMotionGoal()
	goal.motion_name = "home"
	goal.skip_planning = True
	global_play_motion_ac.send_goal(goal)

class localise_self(pt.behaviour.Behaviour):
	# the localization step

	def reset_vars(self):
		# reset all the changed variables
		self.localized = False
		self.move_msg = Twist()
		self.move_msg.angular.z = -1
		self.counter = 0
		self.localization_srv()
		
	def __init__(self, name, loc_srv, clear_cmaps_srv):

		self.localized = False # if we're done with the localization already
		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
		self.move_msg = Twist() # the spinning message
		self.move_msg.angular.z = -1 # what causes the spin
		self.counter = 0
		self.max_count = 60 #the duration of the spin (around one lap)
		self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
		self.localization_srv = loc_srv
		self.localization_srv()

		super(localise_self, self).__init__(name)

	def update(self):
		global head_pose
		global reset_list

		# if already localized, then publish true
		if self.localized and not reset_list[0]:
			return pt.common.Status.SUCCESS

		# Checking if kidnapped and need to relocalize, if so then reset variables
		if reset_list[0]:
			self.reset_vars()
			# clear the "kidnapped"-status
			reset_list[0] = False  

		# If we're looking down, we need to look up to be able to localize ourselves
		if head_pose == "down":
			self.move_head_srv("up")
			rospy.sleep(5)
			head_pose = "up"

		#  otherwise, spin (to look around)
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

			# Now check the particle_cloud with amcl to analyze where we actually are now.
			particle_cloud = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, 5)
			
			#if any element in the particle cloud is 0.05 or smaller, we say that it has converged
			if particle_cloud_is_converged(particle_cloud):
				# set to true if we've localized ourselves
				self.localized = True
				# turn off the reset-need for localization
				reset_list[0] = False
				return pt.common.Status.SUCCESS
			else:
				# otherwise, we need to redo things
				self.reset_vars()
				print("AMCL did not converge! Trying again.")
				return pt.common.Status.FAILURE

def particle_cloud_is_converged(particle_cloud):
	#if the greatest element in the particle cloud is 0.05 or smaller, we say that it has converged

	max_element = 0

	#get the max_element
	for element in particle_cloud.pose.covariance:
		if abs(element) > abs(max_element):
			max_element = element
	
	#check if max_element is smaller than 0.05
	if abs(max_element) < 0.04: # Setting harder demands (will cause spinning, but less tableleg crashing)
		return True 
	return False

class navigation(pt.behaviour.Behaviour):
	# controls the navigation through the map
	def activation_callback(self):
		# activates at the beginning, easier to navigate if head is up.
		print("Beginning navigation...")
		global head_pose
		self.move_head_srv("up")
		head_pose = "up"

	def kidnapped_callback(self, feedback): 
		global reset_list

		# Check if we're kidnapped
		particle_cloud = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, 5)

		if not particle_cloud_is_converged(particle_cloud):
			# we're not done, if we're kidnapped
			self.finished_navigation = False
			self.success_navigation = False

			# Depending on where we are in the sequence, we react differently

			# if we're at the "pick"-navigation, we should first re-localize and then navigate to "pick" again
			if self.goal_string == "pick":
				reset_list[0], reset_list[1] = True, True

			# if we're at the "place"-navigation, we should first re-localize and then navigate to "place" again
			elif self.goal_string == "place":
				# If kidnapped, robot needs to localise itself again.
				reset_list[0], reset_list[3] = True, True
				
			self.move_base_ac.cancel_all_goals()
			# Default is that it is commented in to cancel all goals when this callback is activated.

	def goal_callback(self, state, result):
		# a simple check to see if the navigation was aborted or preempted
		# if so, then we've most likely been kidnapped or hit a wall
		# activate the list of "redo:s"
		if (actionlib.TerminalState.ABORTED or actionlib.TerminalState.PREEMPTED) == state: 
			self.success_navigation = False
			# activate the reset list
			reset_list[0], reset_list[1], reset_list[2], reset_list[3] = True, True, True, True
			try:
				# reset the cube, since it might have disappeared
				global_cube_reset(global_cube_reset_msg)
			except:
				print("Cube was not able to be reset!")

			# reset the arm, so that it doesn't get stuck in objects
			tuckarm_during_run()
			
			# cancel all old goals, since they are no longer relevant
			self.move_base_ac.cancel_all_goals()
		else:
			# Otherwise we have succeeded in navigation and thus set success_navigation=true
			self.success_navigation = True
			self.move_base_ac.cancel_all_goals()

	def __init__(self, name, goal_string, move_base_ac):
		self.finished_navigation = False # a boolean to check if we've already reached the position
		self.goal = None
		self.pick_pose_top = None
		self.place_pose_top = None
		self.move_base_ac = move_base_ac
		self.goal_string = goal_string
		self.goal_msg = MoveBaseGoal()
		self.success_navigation = None # the message from the actionserver (also a boolean)
		self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
		
		# choosing the goal, which is either "pick" or "place"
		if goal_string == 'pick':
			self.pick_pose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
			self.goal = rospy.wait_for_message(self.pick_pose_top, PoseStamped, 5)
			self.goal_msg.target_pose = self.goal
		elif goal_string == 'place':
			self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')
			self.goal = rospy.wait_for_message(self.place_pose_top, PoseStamped, 5)
			self.goal_msg.target_pose = self.goal
		# Otherwise we're trying to reach a non-valid position
		else:
			print("That's not a valid goal.")

		super(navigation, self).__init__(name)


	def reset_vars(self):
		# the reset of variables for the navigation
		self.finished_navigation = False
		self.success_navigation = None

		if self.goal_string == "pick":
			self.goal = rospy.wait_for_message(self.pick_pose_top, PoseStamped, 5)
		else:
			self.goal = rospy.wait_for_message(self.place_pose_top, PoseStamped, 5)
		self.goal_msg.target_pose = self.goal

	def update(self):
		global reset_list
		
		# if we need to redo the pick-navigation and we're in the "pick"-stage
		if reset_list[1] and self.goal_string == "pick":
			# if we've already picked up the cube, then don't reset arm
			# otherwise reset arm, so that we don't crash into things
			if not reset_list[3]:
				tuckarm() 

			# reset variables, so that we can redo our actions
			self.reset_vars()
			# cancel the redo request
			reset_list[1] = False

		# if we need to redo the place-navigation and we're in the "place"-stage
		elif reset_list[3] and self.goal_string == "place":
			# reset variables, so that we can redo our actions
			self.reset_vars()
			# cancel the redo request
			reset_list[3] = False

		# if we've finished without being in a kidnap-event, then always return success
		if self.finished_navigation: 
			if not reset_list[1] and self.goal_string == "pick":
				return pt.common.Status.SUCCESS
			if not reset_list[3] and self.goal_string == "place":
				return pt.common.Status.SUCCESS

		try:
			# set goal to pick or place
			if self.goal_string == "pick":
				self.goal = rospy.wait_for_message(self.pick_pose_top, PoseStamped, 5)

			elif self.goal_string == "place":
				self.goal = rospy.wait_for_message(self.place_pose_top, PoseStamped, 5)

			self.goal_msg.target_pose = self.goal
			self.move_base_ac.send_goal(self.goal_msg, active_cb=self.activation_callback, feedback_cb=self.kidnapped_callback, done_cb=self.goal_callback)
			self.move_base_ac.wait_for_result(rospy.Duration(60.0)) # Default 120.0

			# if we succeeded, set finished navigation to true
			# which stops more attempts at moving and then return true
			if self.success_navigation:
				self.finished_navigation = True
				return pt.common.Status.SUCCESS

			# else return Failure if the navigation didn't work
			elif self.success_navigation == False:
				self.move_base_ac.cancel_goal()
				return pt.common.Status.FAILURE

			else:
				# run until we've succeeded or failed
				return pt.common.Status.RUNNING

		except:
			print('Oh no')

		
class pick_up(pt.behaviour.Behaviour):
	# the "pick up cube" behavior

	def __init__(self, name, pick_srv_nm, aruco_pose_pub):
		self.aruco_pose_pub = aruco_pose_pub
		self.pick_up_srv = rospy.ServiceProxy(pick_srv_nm, SetBool)
		self.picked_up = False # checks if we've already picked up the cube, in which case it won't try further	

		super(pick_up, self).__init__(name)

	def update(self):
		global reset_list

		# check if need to redo cube check/pick_up
		if reset_list[2]:
			reset_list[2] = False
			# if so then reset picked_up, so that we can redo the action
			self.picked_up = False 

		if self.picked_up:
			# always returns true, if we've already picked up the cube and it's not gone
			return pt.common.Status.SUCCESS

		# get the position of the cube
		self.aruco_pose_pub.publish(global_cube_pose)
		# try to pick it up
		print("Attempting to pick up cube.")
		self.pick_up_request = self.pick_up_srv()

		# if success, then set picked up to true, to prevent further attempts
		if self.pick_up_request.success:
			self.picked_up = True
			return pt.common.Status.SUCCESS
		
		elif not self.pick_up_request.success:
			# if failed, then return failure
			return pt.common.Status.FAILURE
		# no "running" behavior will be set for this function
		# as we don't want it to keep on trying to pick_up a non-existent cube

class place(pt.behaviour.Behaviour):
	# the opposite of "pick_up", does essentially the same things however

	def __init__(self, name, place_srv_nm, aruco_pose_pub):
		self.aruco_pose_pub = aruco_pose_pub
		self.place_srv = rospy.ServiceProxy(place_srv_nm, SetBool)
		self.placed = False # checks if, we have done the "place" movement, in which it won't try again

		super(place, self).__init__(name)

	def update(self):
		global reset_list

		if reset_list[4]: 
			self.placed = False 
			# will not add list[4]=False, since the code will finish anyway

		if self.placed:
			# always return success if we've done the movement, to prevent further attempts
			return pt.common.Status.SUCCESS

		self.aruco_pose_pub.publish(global_cube_pose)
		print("Attempting to place cube.")
		self.place_request = self.place_srv()

		if self.place_request.success:
			self.placed = True #if success, then set placed to true
			return pt.common.Status.SUCCESS
		
		elif not self.place_request.success:
			# if this fails, thanks to not actually holding the cube
			# we need to try again

			# reset arm position to prevent crashes
			tuckarm_during_run()

			try:
				# reset cube
				global_cube_reset(global_cube_reset_msg)
			except:
				print("Cube was not able to be reset!")
			
			# now activate the reset list for navigating to pick, cube detection/pick_up and navigation to place
			reset_list[1], reset_list[2], reset_list[3] = True, True, True
			
			# return failure, to go back in the tree
			return pt.common.Status.FAILURE

def set_reset_cube_variable(cmd_vel_top):
	# initiates the "cube"-listener with a predefined message
	# this essentially creates a way to reset cube to it's originall position at any time
	global global_cube_reset_msg
	global global_cube_reset
	global global_cmd_vel_pub

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

	### - Reset Cube position service - ###
	rospy.wait_for_service('/gazebo/set_model_state')
	global_cube_reset = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

	# Instantiate publishers
	global_cmd_vel_pub = rospy.Publisher(cmd_vel_top, Twist, queue_size=10)

class A_level_BehaviourTree(ptr.trees.BehaviourTree):		
	def __init__(self):
		global global_play_motion_ac

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

		### - Reset pose for the cube, to be used later - ###
		set_reset_cube_variable(self.cmd_vel_top)

		# Wait for service providers
		rospy.wait_for_service(self.mv_head_srv_nm, timeout = 30)
		rospy.wait_for_service(self.pick_srv_nm, timeout = 30)
		rospy.wait_for_service(self.place_srv_nm, timeout = 30)
		rospy.wait_for_service(self.localization_srv_nm, timeout = 30)
		rospy.wait_for_service(self.clear_cmaps_srv_nm, timeout = 30)

		# Test of placing the requests for localisation and clear cost maps
		self.localization_req = self.localization_srv()
		self.clear_costmaps_req = self.clear_costmaps_srv()

		# Instantiate publishers
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

		# moving head up and spinning to localize and then activating the navigation
		branch_1 = localise_self("localization", self.localization_srv, self.clear_costmaps_srv)

		# navigation to the "pick" location/position
		branch_2 = navigation("pick navigation", "pick", self.move_base_ac)

		# to detect cube, move head down and then use aruco detection
		branch_3 = detect_cube("detect cube", self.aruco_pose_top)

		# pick up the cube
		branch_4 = pick_up("pick_up", self.pick_srv_nm, self.aruco_pose_pub) # We should not have to publish cube pose again! Dum dum

		# move head up (so that we can "see" the path and walls)
		branch_5 = movehead("up")

		# redoing branch_2 with a new destination (place pose)
		branch_6 = navigation("place navigation", "place", self.move_base_ac)

		# putting down the cube in the same relative pose as we picked it up with
		# of course at the 'place pose' instead of the 'pick pose'
		branch_7 = place("place", self.place_srv_nm, self.aruco_pose_pub) # Move head down?

		# the "final check" as described in canvas, to check whether the cube is there or not
		# if not, redo everything from branch_2 til now
		branch_8 = final_check("final check", self.aruco_pose_top)

		# become the tree
		tree = RSequence(name="Main sequence", children=[branch_0, branch_1, branch_2, branch_3, branch_4, branch_5, branch_6, branch_7, branch_8])
		super(A_level_BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)
		



if __name__ == "__main__":

	rospy.init_node('main_behaviour_tree')
	try:
		A_level_BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
