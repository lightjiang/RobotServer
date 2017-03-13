# coding=utf-8
"""
20170210
light
接受MQ消息的ros节点
"""
from script.MQhandler import MQListen
from script.base import MotionHandler
import json
import os
import time
import django
import rospy
from std_srvs.srv import Empty
os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'RobotsServer.settings')
django.setup()
from core.models import MarkNode, Robot


class MQNode(MQListen):
    def __init__(self):
        super(MQNode, self).__init__()
        self.robot_handler = MotionHandler()

    def run(self, ch, method, properties, body):
        body = json.loads(body)
        print(body)
        if "no_ack" in body:
            no_ack = body["no_ack"]
        else:
            no_ack = True
        cat = body["category"].lower()
        func = getattr(self, cat)
        delay = time.time() - body["timestamp"]
        if delay <= body["delay"]:
            if not no_ack:
                self.response(ch, method, properties, func(**body))
            else:
                func(**body)
        else:
            print("timeout, mission cancel")
        ch.basic_ack(delivery_tag=method.delivery_tag)

    def goalmove(self, **kwargs):
        self.robot_handler.goal(**kwargs)
        return True

    def update_status(self, **kwargs):
        robot = Robot.objects.get(id=kwargs["robot_id"])
        stat = self.robot_handler.move_base.get_goal_status_text()
        print([stat])
        if stat == "":
            mode = 0
        elif stat == "Failed to find a valid plan. Even after executing recovery behaviors.":
            self.clear_costmaps()
            mode = 1
        elif stat == "This goal has been accepted by the simple action server":
            mode = 1
        elif stat == 'ERROR: Called get_goal_status_text when no goal is running':
            mode = 0
        elif stat == "Goal reached.":
            mode = 0
        else:
            mode = 0
        if mode != robot.on_mission:
            robot.on_mission = mode
            robot.save()
        return 1

    def basemove(self, **kwargs):
        self.robot_handler.move(**kwargs)

    def marknode(self, **kwargs):
        robot = Robot.objects.get(id=kwargs["robot_id"])
        node = MarkNode(robot=robot, label=kwargs["label"], map_name=robot.local_map)
        (position, rotation) = self.robot_handler.get_coordinate()
        node.x = position.x
        node.y = position.y
        node.rz = rotation
        node.my_save()
        return {"nodeId": node.id}

    @staticmethod
    def clear_costmaps(**kwargs):
        rospy.wait_for_service('/move_base/clear_costmaps')
        a = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        a.call()

    def cancel_all_goal(self, **kwargs):
        self.robot_handler.move_base.cancel_all_goals()
        return 1

    def back_dock(self):
        return self.robot_handler.back_dock()

if __name__ == '__main__':
    MQNode().listen()
