# coding=utf-8
"""
light
20170313
"""
from gevent import monkey
from MQNode import MQNode
from MQhandler import MQListen
from script.base import MotionHandler
import gevent
monkey.patch_all()


def mq_node(robot_handler):
    print robot_handler
    MQNode().listen()


def main():
    pool = []
    # robot_handler = MotionHandler()
    for i in range(3):
        pool.append(gevent.spawn(mq_node, i))
    gevent.joinall(pool)

if __name__ == '__main__':
    main()
