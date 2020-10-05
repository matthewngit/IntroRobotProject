#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence
from geometry_msgs.msg import PoseWithCovarianceStamped

import numpy as np

def print_tree(t):
    print("\033[2J\033[H")
    pt.display.print_ascii_tree(t, show_status=True)

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

                #pose = rospy.get_param()

		rospy.loginfo("Initialising behaviour tree")

                localization = pt.composites.Selector("Localization", children=[
                    position_is_known("Position known?"),
                    go("Turn", 0, 1)
                    ])

                place_and_finish = RSequence("Place and finish", children=[
                    is_at_table2("Is at table 2?"),
                    pt.composites.Selector("Visible or place", children=[
                        RSequence("Check cube", [
                            movehead("Lower head", "down"),
                            cube_visible("Cube visible?"),
                            ]),
                        RSequence("Place", children=[
                            is_holding_cube("Holding cube?"),
                            pick_place("Place", False),
                            ])
                        ])
                    ])

                move_to_table2 = pt.composites.Selector("Move to table 2", children=[
                    RSequence("Should go to table 2?", children=[
                        is_at_table2("Is at table 2?"),
                        is_holding_cube("Holding cube?")
                        ]),
                    RSequence("Move to table 2", children=[
                        is_holding_cube("Holding cube?"),
                        movehead("Raise head", "up"),
                        move_to_pose("Move to table 2", 2),
                        ])
                    ])

                pick = RSequence("Pick cube", children=[
                    is_at_table1("Is at table 1?"),
                    movehead("Lower head", "down"),
                    cube_visible("Cube visible?"),
                    pick_place("Pick", True),
                    ])

                move_to_table1 = pt.composites.Selector("Move to 1", children=[
                    is_at_table1("Is at table 1?"),
                    RSequence("Move to table 1", children=[
                        movehead("Raise head", "up"),
                        tuckarm(True),
                        move_to_pose("Move to table 1", 1),
                        ])
                    ])

		# become the tree
		tree = RSequence(name="Main sequence", children=[
                    tuckarm(),
                    localization,
                    pt.composites.Selector("Main behaviour", children=[
                        place_and_finish,
                        move_to_table2,
                        pick,
                        move_to_table1
                        ])
                    ])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown():
                    self.tick_tock(1, post_tick_handler=lambda t: print_tree(tree))

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
