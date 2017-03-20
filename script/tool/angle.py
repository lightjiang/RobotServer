# coding=utf-8
"""
light
20170315
"""
import PyKDL
from math import pi
from geometry_msgs.msg import Twist, Point, Quaternion


def quat_to_angle(quat):
    rot = PyKDL.Rotation().Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]


def angle_to_quat(angle):
    rot = PyKDL.Rotation().RPY(0, 0.0, angle)
    temp = rot.GetQuaternion()
    return Quaternion(**{"x": temp[0], "y": temp[1], "z": temp[2], "w": temp[3]})


def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

if __name__ == '__main__':
    print([angle_to_quat(0)])
