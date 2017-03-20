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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal
from actionlib_msgs.msg import GoalStatus
roslib.load_manifest('kobuki_auto_docking')


def done_cb(status, result):
    if status == GoalStatus.PENDING:
        state = 'PENDING'
    elif status == GoalStatus.ACTIVE:
        state = 'ACTIVE'
    elif status == GoalStatus.PREEMPTED:
        state = 'PREEMPTED'
    elif status == GoalStatus.SUCCEEDED:
        state = 'SUCCEEDED'
    elif status == GoalStatus.ABORTED:
        state = 'ABORTED'
    elif status == GoalStatus.REJECTED:
        state = 'REJECTED'
    elif status == GoalStatus.PREEMPTING:
        state = 'PREEMPTING'
    elif status == GoalStatus.RECALLING:
        state = 'RECALLING'
    elif status == GoalStatus.RECALLED:
        state = 'RECALLED'
    elif status == GoalStatus.LOST:
        state = 'LOST'
    back_dock()
    # Print state of action server


class MotionHandler(object):
    def __init__(self, node="motion_base_control", mode=1):
        rospy.init_node(node, anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
        self.tf_listener = tf.TransformListener()
        self.map_frame = "/map"
        self.odom_frame = '/odom'
        self.base_frame = '/base_footprint'
        self.on_dock = True
        if mode:
            self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(1.1))
        else:
            self.tf_listener.waitForTransform(self.map_frame, self.base_frame, rospy.Time(), rospy.Duration(2))

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
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x  # 3 meters
        goal.target_pose.pose.position.y = y  # 3 meters
        goal.target_pose.pose.orientation = Quaternion()
        goal.target_pose.pose.orientation = angle_to_quat(rz)
        if x==0 and y == 0 and rz == 0:
            print 1423
            self.move_base.send_goal(goal, done_cb=done_cb)
        else:
            print(1111)
            self.move_base.send_goal(goal)

    def test(self):
        self.goal(x=-4.58, y=-4.42)  # -math.pi/2)
        # self.goal()

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
        (position, rotation) = self.get_odom()
        last_x = position.x
        last_y = position.y
        last_angle = rotation
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
            (position, rotation) = self.get_odom()
            delta_angle = normalize_angle(rotation - last_angle)
            turn_angle += delta_angle
            last_angle = rotation
            distance += math.sqrt(pow((position.x - last_x), 2) + pow((position.y - last_y), 2))
            last_x = position.x
            last_y = position.y
            if distance >= abs(x):
                move_cmd.linear.x = 0.0
            if abs(turn_angle) >= abs(angle):
                move_cmd.angular.z = 0.0
            if not (move_cmd.linear.x or move_cmd.angular.z):
                stat = 0

    def get_odom(self):
        (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        return Point(*trans), quat_to_angle(Quaternion(*rot))

    def get_coordinate(self):
        (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        res = Point(*trans), quat_to_angle(Quaternion(*rot))
        return res

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def back_dock(self):
        back_dock()
        self.on_dock = True
        # client.wait_for_result()
        # if client.get_result().text == "Arrived on docking station successfully.":
        #     self.on_dock = True
        #     return True
        # else:
        #     return client.get_result()


def back_dock():
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
    handler.on_dock = False
    handler.goal(0, 0, 0)
    print handler.get_coordinate()


