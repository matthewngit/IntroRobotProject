# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.


import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, SetBool
import actionlib
import numpy as np

data = {}
data["table2"] = False
data["table1"] = False
data["holding_cube"] = False
data["tuck_arm"] = True

class move_to_pose(pt.behaviour.Behaviour):

    def __init__(self, name, table):
        self.action_nm = rospy.get_param(rospy.get_name() + "/nav_goal_topic")
        self.navigate = actionlib.SimpleActionClient(self.action_nm, MoveBaseAction)
        self.navigate.wait_for_server()

        self.table1_pose_top = rospy.get_param(rospy.get_name() + "/pick_pose_topic")
        self.table2_pose_top = rospy.get_param(rospy.get_name() + "/place_pose_topic")
        self.table1_sub = rospy.Subscriber(self.table1_pose_top, PoseStamped, callback=self.cache_table1)
        self.table2_sub = rospy.Subscriber(self.table2_pose_top, PoseStamped, callback=self.cache_table2)

        self.table1_pose = None
        self.table2_pose = None

        self.table = table

        self.done = False
        self.begun = False
        self.status = None

        super(move_to_pose, self).__init__(name)

    def update(self):
        global data

        if self.done:
            self.done = False
            self.begun = False

            if self.status == pt.common.Status.SUCCESS:
                data["table2"] = self.table == 2
                data["table1"] = self.table == 1
            else:
                data["table2"] = False
                data["table1"] = False

            return self.status

        if not self.begun:
            data["table2"] = False
            data["table1"] = False
            goal = None
            if self.table == 1:
                goal = MoveBaseGoal(self.table1_pose)
            elif self.table == 2:
                goal = MoveBaseGoal(self.table2_pose)

            self.navigate.send_goal(goal, done_cb=self.done_callback)
            self.begun = True
            return pt.common.Status.RUNNING

        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        self.navigate.cancel_all_goals()
        self.done = False
        self.begun = False

    def cache_table1(self, resp):
        self.table1_pose = resp

    def cache_table2(self, resp):
        self.table2_pose = resp

    def done_callback(self, state, result):
        self.done = True
        if state == actionlib.TerminalState.SUCCEEDED:
            self.status = pt.common.Status.SUCCESS
        else:
            self.status = pt.common.Status.FAILURE

class pick_place(pt.behaviour.Behaviour):

    def __init__(self, name, pick):
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')

        if pick:
            rospy.wait_for_service(self.pick_srv_nm, timeout=30)
        else:
            rospy.wait_for_service(self.place_srv_nm, timeout=30)

        self.pick = pick

        # become a behaviour
        super(pick_place, self).__init__(name)

    def update(self):
        global data

        srv = rospy.ServiceProxy(self.pick_srv_nm if self.pick else self.place_srv_nm, SetBool)
        if srv(False).success:
            data["holding_cube"] = self.pick
            if not self.pick:
                data["tuck_arm"] = True
            rospy.sleep(1)
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class cube_visible(pt.behaviour.Behaviour):

    def __init__(self, name):
        self.marker_pose_nm = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

        self.sub = rospy.Subscriber(self.marker_pose_nm, PoseStamped, callback=self.cache_message)
        self.newest_msg_time = rospy.Time(0)

        # become a behaviour
        super(cube_visible, self).__init__(name)
                                                
    def update(self):
        if rospy.Time.now() - self.newest_msg_time < rospy.Duration.from_sec(1):
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def cache_message(self, msg):
        self.newest_msg_time = msg.header.stamp

class is_holding_cube(pt.behaviour.Behaviour):

    def __init__(self, name):
        # become behaviour
        super(is_holding_cube, self).__init__(name)

    def update(self):
        if data["holding_cube"]:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class is_at_table2(pt.behaviour.Behaviour):

    def __init__(self, name):
        # become behaviour
        super(is_at_table2, self).__init__(name)

    def update(self):
        if data["table2"]:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class is_at_table1(pt.behaviour.Behaviour):

    def __init__(self, name):
        # become behaviour
        super(is_at_table1, self).__init__(name)

    def update(self):
        if data["table1"]:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class position_is_known(pt.behaviour.Behaviour):

    def __init__(self, name):
        self.pose_top = rospy.get_param(rospy.get_name() + "/amcl_estimate")
        self.clear_nm = rospy.get_param(rospy.get_name() + "/clear_costmaps_srv")
        self.global_loc_nm = rospy.get_param(rospy.get_name() + "/global_loc_srv")

        rospy.wait_for_service(self.clear_nm)
        rospy.wait_for_service(self.global_loc_nm)

        self.sub = rospy.Subscriber(self.pose_top, PoseWithCovarianceStamped, callback=self.cache_covariance)

        self.cov = np.zeros((6, 6))
        self.threshold = 0.035

        self.became_fail = False

        # become a behaviour
        super(position_is_known, self).__init__(name)

    def update(self):
        if max(np.linalg.eigvals(self.cov)) > self.threshold:
            if not self.became_fail:
                clear = rospy.ServiceProxy(self.clear_nm, Empty)
                global_loc = rospy.ServiceProxy(self.global_loc_nm, Empty)

                clear()
                global_loc()

                self.became_fail = True
            return pt.common.Status.FAILURE
        else:
            self.became_fail = False
            return pt.common.Status.SUCCESS

    def cache_covariance(self, resp):
        self.cov = np.reshape(resp.pose.covariance, (6, 6))

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

    def __init__(self, inside_loop=False):

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

        self.inside_loop = inside_loop

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):
        if data["tuck_arm"] and self.inside_loop:
            self.sent_goal = False
            self.finished = False
            data["tuck_arm"] = False

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
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raisesthe head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, name, direction):

        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # become a behaviour
        super(movehead, self).__init__(name)

    def update(self):
        self.move_head_req = self.move_head_srv(self.direction)
        return pt.common.Status.SUCCESS
