#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

def print_tree(t):
    print("\033[2J\033[H")
    pt.display.print_ascii_tree(t, show_status=True)

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

                b0 = RSequence(name="Prepare pick", children=[tuckarm(), movehead("down")])

                b1 = pick_place(True, "Pick cube")

                b2 = RSequence(name="Move to table 2", children=[
                    set_data("moving", True, "Moving to table 2"),
                    pt.composites.Selector("Turn", children=[
                        counter(30, "Turn counter"),
                        go("Rotate", 0, -1)
                        ]),
                    pt.composites.Selector(name="Forward", children=[
                        counter(9, "Forward counter"),
                        go("Move", 0.8, 0)
                        ]),
                    ])

                b3 = pick_place(False, "Place cube")

                moved_to_2 = RSequence(name="Moved 2 2", children=[
                    set_data("table1", False, "Moved to table 2"),
                    set_data("moving", False, "Arrived at table 2")
                    ])

                pick_move_place = RSequence(name="Pick, move and place", children=[is_at_table1("Is at table 1"), b0, b1, b2, b3, moved_to_2])

                done = is_done("We are maybe done folks")

                go_back = RSequence(name="Go back", children=[
                    set_data("moving", True, "Moving to table 1"),
                    pt.composites.Selector("Turn", children=[
                        counter(30, "Turn counter"),
                        go("Rotate", 0, -1)
                        ]),
                    pt.composites.Selector(name="Forward", children=[
                        counter(9, "Forward counter"),
                        go("Move", 0.8, 0)
                        ]),
                    set_data("table1", True, "Moved to table 1"),
                    set_data("moving", False, "Arrived at table 1"),
                    reset
                    ])

		## go to door until at door
		#b0 = pt.composites.Selector(
		#	name="Go to door fallback", 
		#	children=[counter(30, "At door?"), go("Go to door!", 1, 0)]
		#)

		## tuck the arm
		#b1 = tuckarm()

		## go to table
		#b2 = pt.composites.Selector(
		#	name="Go to table fallback",
		#	children=[counter(5, "At table?"), go("Go to table!", 0, -1)]
		#)

		## move to chair
		#b3 = pt.composites.Selector(
		#	name="Go to chair fallback",
		#	children=[counter(13, "At chair?"), go("Go to chair!", 1, 0)]
		#)

		## lower head
		#b4 = movehead("down")d


		## become the tree
		#tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
                # become the tree
                tree = pt.composites.Selector(name="Main tree", children=[pick_move_place, done, go_back])
                
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown():
                    self.tick_tock(1, post_tick_handler=lambda t: print_tree(tree))

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
        #pt.logging.level = pt.logging.Level.DEBUG
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
