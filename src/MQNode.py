# coding=utf-8
"""
20170210
light
接受MQ消息的ros节点
"""
from src.MQhandler import MQListen
from src.base import MotionHandler
import json
import os
import django
os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'RobotsServer.settings')
django.setup()
from core.models import MarkNode


class MQNode(MQListen):
    def __init__(self):
        super(MQNode, self).__init__()
        self.robot_handler = MotionHandler()

    def run(self, ch, method, properties, body):
        print(body)
        body = json.loads(body)
        cat = body["category"].lower()
        func = getattr(self, cat)
        func(**body)
        ch.basic_ack(delivery_tag=method.delivery_tag)

    def goalmove(self, **kwargs):
        self.robot_handler.goal(**kwargs)

    def basemove(self, **kwargs):
        self.robot_handler.move(**kwargs)

    def marknode(self, **kwargs):
        node = MarkNode(robot_id=kwargs["robot_id"], label=kwargs["label"])
        (position, rotation) = self.robot_handler.get_coordinate()
        node.x = position.x
        node.y = position.y
        node.rz = rotation
        node.my_save()

if __name__ == '__main__':
    MQNode().listen()
