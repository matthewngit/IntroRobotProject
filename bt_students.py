#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

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

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
