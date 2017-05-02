# coding=utf-8
"""
light
2016.12.02
turtlebot motion control
"""
import rospy
import roslib
import tf
import math
import actionlib
from geometry_msgs.msg import Twist, Point, Quaternion
from tool.angle import quat_to_angle, normalize_angle, angle_to_quat
from std_srvs.srv import Empty
from fetch_move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from fetch_auto_dock_msgs.msg import DockAction, DockGoal, UndockAction, UndockGoal
from actionlib_msgs.msg import GoalStatus
roslib.load_manifest('kobuki_auto_docking')


class MotionHandler(object):
    def __init__(self, node="motion_base_control", mode=1):
        rospy.init_node(node, anonymous=False)
        rospy.on_shutdown(self.shutdown)
        # self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # self.move_base.wait_for_server(rospy.Duration(5))
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.tf_listener = tf.TransformListener()
        self.map_frame = "map"
        self.odom_frame = '/odom'
        self.base_frame = 'base_link'
        # self.tf_listener.waitForTransform(self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(1.2))


    def init_dock(self):
        self.dock_client = actionlib.SimpleActionClient('dock_drive_action', AutoDockingAction)
        while not self.dock_client.wait_for_server(rospy.Duration(5)):
            if rospy.is_shutdown():
                return
            print 'Action server is not connected yet. still waiting...'

    @staticmethod
    def clear_costmaps():
        rospy.wait_for_service('/move_base/clear_costmaps')
        a = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        a.call()

    def goal(self, x=0.0, y=0.0, rz=0.0, **kwargs):
        if self.on_dock:
            self.on_dock = False
            self.move(x=-0.2)
            self.clear_costmaps()
        else:
            self.dock_client.cancel_all_goals()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x  # 3 meters
        goal.target_pose.pose.position.y = y  # 3 meters
        goal.target_pose.pose.orientation = Quaternion()
        goal.target_pose.pose.orientation = angle_to_quat(rz)
        if x == 0 and y == 0 and rz == 0:
            self.move_base.send_goal(goal, done_cb=self.back_callback)
        else:
            self.move_base.send_goal(goal)

    def move(self, x=0.0, angle=0.0, hz=10, speed=0.2, rotate=1.0, **kwargs):
        """
        :param x: 前进或后退距离
        :param angle: 旋转角度  正为左转
        :param hz: 指令发送频率
        :param speed: 行进速度
        :param rotate: 旋转速度
        :return:
        """
        move_cmd = Twist()
        r = rospy.Rate(hz)
        if x:
            move_cmd.linear.x = speed * x / abs(x)
        if angle:
            move_cmd.angular.z = rotate * angle / abs(angle)
        stat = 1
        turn_angle = 0
        distance = 0
        while not rospy.is_shutdown() and stat:
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            stat += 1
            if stat>10:
                stat = 0


    def get_coordinate(self):
        (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        #(trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        res = Point(*trans), quat_to_angle(Quaternion(*rot))
        return res

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        #self.dock_client.cancel_all_goals()
        #self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def back_dock(self, *args, **kwargs):
        print "docking :", args, kwargs
        goal = AutoDockingGoal()
        self.dock_client.send_goal(goal, done_cb=self.dock_callback)
        # client.wait_for_result()
        # if client.get_result().text == "Arrived on docking station successfully.":
        #     self.on_dock = True
        #     return True
        # else:
        #     return client.get_result()

    def dock_callback(self, status, *args, **kwargs):
        print("dock status:", status)
        if status == 3:
            self.on_dock = True
        else:
            self.dock_client.cancel_all_goals()

    def back_callback(self, status, *args, **kwargs):
        print("back status :", status)
        if status == 3:
            self.back_dock()


def back_dock(*args, **kwargs):
    client = actionlib.SimpleActionClient('dock_drive_action', AutoDockingAction)
    while not client.wait_for_server(rospy.Duration(5)):
        if rospy.is_shutdown():
            return
        print 'Action server is not connected yet. still waiting...'

    goal = AutoDockingGoal()
    client.send_goal(goal)
    rospy.on_shutdown(client.cancel_goal)
    return client


if __name__ == '__main__':
    handler = MotionHandler(mode=0)
    handler.move(x=-0.3)
    #print handler.get_coordinate()
    # root:(14.606216067345356, 11.950999296165001, 0.4246052772385264)

