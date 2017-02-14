# coding=utf-8
from socket import *
from threading import Thread
import time
import traceback
import json
BUFSIZ = 1024*100


class Server(object):
    def __init__(self):
        self.host = "0.0.0.0"
        self.port = 17000
        self.coreserver = socket(AF_INET, SOCK_STREAM)
        self.coreserver.bind((self.host, self.port))
        self.coreserver.listen(10)
        self.connections = {"0": {}, "1": {}}   # 0: commandclient      1: actionclient
        while True:
            client, addre = self.coreserver.accept()
            ConnectHandler(client=client).start()
        self.coreserver.close()


class ConnectHandler(Thread):
    def __init__(self, client):
        Thread.__init__(self)
        self.client = client
        self.stat = 1

    def run(self):
        while self.stat:
            try:
                content = self.client.recv(BUFSIZ)
                if content:
                    self.handler(content)
            except:
                traceback.print_exc()
                self.stat = 0
                self.client.close()
                print("disconnected")
                break
            time.sleep(0.01)

    def handler(self, content):
        print(content)
        content = json.loads(content)
        if content is not dict:
            for msg in ["category", "key", "userId"]:
                if msg not in content:
                    self.send(status=1002, response_text="data structure error: missing %s" % msg)
                    return
            cat = content["category"]
            if cat == "GetAddressNodeList":
                self.send(tableList=["T1", "T2"])
            elif cat == "GetRobotsList":
                robots = [
                    {
                        "robotId": "102",
                        "robotName": "hel"
                    }
                ]
                self.send(robotList=robots)
            elif cat == "GetRobotsStatus":
                pass
            elif cat == "ExecuteMission":
                self.send()
            elif cat == "CancelMission":
                self.send()
            else:
                self.send(status=1003, response_text="api %s not exist" % cat)
        else:
            self.send(status=1001, response_text="data structure error")

    def send(self, status=0, response_text=None, **kwargs):
        if not response_text:
            if not status:
                response_text = u"请求成功"
            else:
                response_text = u"请求失败"
        res = {
            "status": status,
            "responseText": response_text,
               }
        res.update(**kwargs)
        print(res)
        try:
            self.client.send(json.dumps(res))
        except:
            pass

if __name__ == '__main__':
    Server()
