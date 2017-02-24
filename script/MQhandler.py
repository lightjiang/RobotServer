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

    def declare(self, queue="BaseMission", durable=True, **kwargs):
        # queue:
        #       0:BaseMission
        #       1:VitalMission
        #       2:EmergencyMission
        self.channel.queue_declare(queue=queue, durable=durable, **kwargs)


class MQSend(MQBase):
    def __init__(self):
        super(MQSend, self).__init__()

    def on_response(self, ch, method, props, body):
        if self.corr_id == props.correlation_id:
            self.response = body

    def send(self, queue="BaseMission", message=None, durable=True):
        self.declare(queue=queue, durable=durable)
        if "no_ack" in message:
            no_ack = message["no_ack"]
        else:
            no_ack = True
        if not no_ack:
            result = self.channel.queue_declare(exclusive=True)
            callback_queue = result.method.queue
            self.channel.basic_consume(self.on_response, no_ack=True,
                                       queue=callback_queue)
            self.response = None
            self.corr_id = str(uuid.uuid4())
            self.channel.basic_publish(exchange='', routing_key=queue,  properties=pika.BasicProperties(
                                             reply_to=callback_queue,
                                             correlation_id=self.corr_id,
                                             ), body=json.dumps(message))
            while self.response is None:
                self.connection.process_data_events()
            return self.response
        else:
            self.channel.basic_publish(exchange='', routing_key=queue, body=json.dumps(message))

    @staticmethod
    def message(category, delay=1, no_ack=True, **kwargs):
        message = {"category": category, "no_ack": no_ack, "userId": "test", "key": "test",
                   "timestamp": time.time(), "delay": delay}
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

    @staticmethod
    def response(ch, method, properties, body):
        ch.basic_publish(exchange='',
                         routing_key=properties.reply_to,
                         properties=pika.BasicProperties(correlation_id=properties.correlation_id),
                         body=json.dumps(body))

if __name__ == '__main__':
    pass
