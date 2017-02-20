# coding=utf-8
"""
20170210
light
rabbitmq发送和接受消息封装函数
"""
import pika
import time
import json
import uuid


class MQBase(object):
    def __init__(self):
        self.connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
        self.channel = self.connection.channel()

    def declare(self, queue="BaseMission", durable=True):
        # queue:
        #       0:BaseMission
        #       1:VitalMission
        #       2:EmergencyMission
        self.channel.queue_declare(queue=queue, durable=durable)


class MQSend(MQBase):
    def __init__(self):
        super(MQSend, self).__init__()

    def send(self, queue="BaseMission", message=None, durable=True):
        self.declare(queue=queue, durable=durable)
        self.channel.basic_publish(exchange='', routing_key=queue, body=json.dumps(message))

    @staticmethod
    def message(category, **kwargs):
        message = {"category": category, "userId": "test", "key": "test", "timestamp": time.time()}
        message.update(kwargs)
        return message


class MQListen(MQBase):
    def __init__(self):
        super(MQListen, self).__init__()

    def listen(self, queue="BaseMission", no_ack=False, durable=True):
        self.declare(queue=queue, durable=durable)
        self.channel.basic_qos(prefetch_count=1)
        self.channel.basic_consume(self.run, queue=queue, no_ack=no_ack)
        self.channel.start_consuming()

    def run(self, *args, **kwargs):
        pass

if __name__ == '__main__':
    pass
