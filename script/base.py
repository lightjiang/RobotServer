# coding=utf-8
"""
light
2016.12.02
turtlebot motion control
"""
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import pika
from actionlib_msgs.msg import *


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
        if mode:
            self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(1.1))
        else:
            self.tf_listener.waitForTransform(self.map_frame, self.base_frame, rospy.Time(), rospy.Duration(2))

    def goal(self, x=0.0, y=0.0, **kwargs):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x  # 3 meters
        goal.target_pose.pose.position.y = y  # 3 meters
        goal.target_pose.pose.orientation.w = 1.0  # go forward
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

if __name__ == '__main__':
    import time
    for i in range(10):
        a = time.time()
        handler = MotionHandler(node="test", mode=0)
        #print(time.time())
        #print(handler.get_coordinate())
        print(time.time() - a)
    # handler.goal(x=1.9614, y=-0.56)
