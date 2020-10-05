# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.


import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist, PoseStamped
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from std_srvs.srv import SetBool
from reactive_sequence import *

data = {}
data["moving"] = False
data["table1"] = True
data["tucked_arm"] = False
data["lowered_head"] = False
data["picked"] = False
data["placed"] = False
data["moved"] = False


class is_at_table1(pt.behaviour.Behaviour):

    def __init__(self, name):
        # become behaviour
        super(is_at_table1, self).__init__(name)

    def update(self):
        if data["table1"]:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class set_data(pt.behaviour.Behaviour):

    def __init__(self, key, value, name):
        self.key = key
        self.value = value
        
        # become behaviour
        super(set_data, self).__init__(name)

    def update(self):
        global data

        data[self.key] = self.value
        return pt.common.Status.SUCCESS

class is_done(pt.behaviour.Behaviour):

    def __init__(self, name):
        self.marker_pose_nm = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

        # become a behaviour
        super(is_done, self).__init__(name)

    def update(self):
        if data["moving"]:
            return pt.common.Status.FAILURE

        try:
            rospy.wait_for_message(self.marker_pose_nm, PoseStamped, 1)
            return pt.common.Status.SUCCESS
        except rospy.ROSException:
            return pt.common.Status.FAILURE

class pick_place(pt.behaviour.Behaviour):

    def __init__(self, pick, name):
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')

        if pick:
            rospy.wait_for_service(self.pick_srv_nm, timeout=30)
        else:
            rospy.wait_for_service(self.place_srv_nm, timeout=30)

        self.pick = pick

        self.finished = False
        self.success = False

        # become a behaviour
        super(pick_place, self).__init__(name)

    def update(self):
        global data

        if not data["picked"] and self.pick:
            self.finished = False
            self.success = False
            data["picked"] = True
        
        if not data["placed"] and not self.pick:
            self.finished = False
            self.success = False
            data["placed"] = True

        if self.finished:
            return pt.common.Status.SUCCESS if self.success else pt.common.Status.FAILURE

        else:
            srv = rospy.ServiceProxy(self.pick_srv_nm if self.pick else self.place_srv_nm, SetBool)
            self.success = srv(False).success
            self.finished = True
            return pt.common.Status.RUNNING

class counter(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(counter, self).__init__(name)

    def update(self):
        global data

        # increment i
        self.i += 1

        # succeed after count is done
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS


class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        rospy.loginfo("Initialising go behaviour.")

        # action space
        #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # become a behaviour
        super(go, self).__init__(name)

    def update(self):

        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()

        # tell the tree that you're running
        return pt.common.Status.RUNNING

class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):
        global data

        if not data["tucked_arm"]:
            self.finished = False
            self.sent_goal = False
            data["tucked_arm"] = True
        
        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        #elif not self.play_motion_ac.get_result():
        #    rospy.loginfo("Jag har failat")
        #    return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raisesthe head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movehead, self).__init__("Lower head!")

    def update(self):
        global data
        
        if not data["lowered_head"]:
            self.done = False
            self.tried = False
            data["lowered_head"] = True
        
        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:
            # tell the tree you're running
            self.tried = True
            return pt.common.Status.RUNNING

        if self.tried:
            self.move_head_srv(self.direction)
            self.done = True
            return pt.common.Status.SUCCESS

reset = RSequence("Reset data", [
    set_data("tucked_arm", False, "Reset tuck"),
    set_data("lowered_head", False, "Reset lowered"),
    set_data("picked", False, "Reset picked"),
    set_data("placed", False, "Reset placed")
    ])
