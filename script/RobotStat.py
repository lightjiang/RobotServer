# coding=utf-8
"""
light
20170317
机器人状态检测节点
"""

import rospy
from kobuki_msgs.msg import SensorState
from actionlib_msgs.msg import GoalStatusArray
import os
import django
os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'RobotsServer.settings')
django.setup()
from core.models import Robot


class RobotStat(object):
    def __init__(self):
        self.robot = Robot.objects.get(id="T001")
        rospy.init_node("robot_stat")
        rospy.Subscriber("/mobile_base/sensors/core", SensorState, self.robot_power)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.mission_stat)
        self.kobuki_base_max_charge = 162
        rospy.spin()

    def robot_power(self, data):
        per = int(round(float(data.battery) / float(self.kobuki_base_max_charge) * 100))
        self.robot.charge = per
        if (int(data.charger) == 0):
            self.robot.on_dock = 0
        else:
            self.robot.on_dock = 1
        self.robot.save()

    def mission_stat(self, data):
        stat = data.status_list[0].status
        if stat == 3:
            self.robot.on_mission = True
            self.robot.save()

if __name__ == '__main__':
    RobotStat()
