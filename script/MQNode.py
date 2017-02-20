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
os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'RobotsServer.settings')
django.setup()
from core.models import MarkNode, Robot


class MQNode(MQListen):
    def __init__(self):
        super(MQNode, self).__init__()
        self.robot_handler = MotionHandler()

    def run(self, ch, method, properties, body):
        print(body)
        body = json.loads(body)
        timestamp = body["timestamp"]
        cat = body["category"].lower()
        func = getattr(self, cat)
        func(**body)
        ch.basic_ack(delivery_tag=method.delivery_tag)

    def goalmove(self, **kwargs):
        self.robot_handler.goal(**kwargs)

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

if __name__ == '__main__':
    MQNode().listen()
