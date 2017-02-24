# coding=utf-8
from core.models import MarkNode, Robot
from script.MQhandler import MQSend


class ApiHandler(object):
    def __init__(self, handler):
        self.handler = handler
        self.post = self.handler.post
        self.get = self.handler.get
        self.request = self.handler.request
        self.session = self.handler.session
        self.file = self.request.FILES

    def get_address_node_list(self):
        temp = {}
        robots = Robot.objects.all()
        for robot in robots:
            temp[robot.id] = []
            for node in robot.marknode_set.filter(status=True, map_name=robot.local_map):
                tem = {"NodeName": node.label, "NodeId": node.id}
                temp[robot.id].append(tem)
        return self.__response(tableList=temp)

    def get_robots_list(self):
        self.update_status()
        from core.models import Robot
        robots = Robot.objects.all()
        temp = []
        for i in robots:
            dic = {
                "robotId": i.id,
                "robotName": i.name,
                "on_mission": i.on_mission,
                "charge": i.charge,
                "node": []
            }
            for node in i.marknode_set.filter(status=True):
                tem = {"NodeName": node.label, "NodeId": node.id}
                dic["node"].append(tem)
            temp.append(dic)

        return self.__response(robotList=temp)

    def get_robots_status(self):
        self.update_status()
        data = self.post
        if "robotId" in data:
            temp = Robot.objects.filter(id=data["robotId"].upper())
            if temp:
                robot = temp[0]
                stat = {
                    "on_mission": robot.on_mission,
                    "charge": robot.charge,
                    "node": []
                }
                for node in robot.marknode_set.filter(status=True):
                    tem = {"NodeName": node.label, "NodeId": node.id}
                    stat["node"].append(tem)
                return self.__response(**stat)
            else:
                return self.__response(status=2001, response_text="robot not exist")
        else:
            return self.__response(status=1002, response_text="argument error: missing %s" % "robotId")

    @staticmethod
    def update_status(robot_id="T001"):
        mq = MQSend()
        ms = mq.message("update_status", robot_id=robot_id, no_ack=False)
        mq.send(message=ms)
        return True

    def update_node(self):
        """
        更新标记点信息接口
        category: 0:删、1:增
        robotId: 机器人编号, 0,1有效
        nodeId： 节点编号, 0有效
        label： 1有效

        :return: 127.0.0.1:8000/api/update_node/?robotId=t001&category=0&label=test2&nodeId=2
        """
        data = self.post
        if "robotId" in data and "category" in data:
            category = data["category"]
            temp = Robot.objects.filter(id=data["robotId"].upper())
            if temp:
                robot = temp[0]
                if category == "0":
                    nodes = MarkNode.objects.filter(id=int(data["nodeId"]), status=True)
                    if nodes:
                        node = nodes[0]
                        node.status = False
                        node.my_save()
                        return self.__response()
                    else:
                        self.__response(status=2011, response_text="node %s not existed" % data["nodeId"])
                elif category == "1":
                    if MarkNode.objects.filter(robot=robot, label=data["label"], status=True, map_name=robot.local_map):
                        return self.__response(status=2012, response_text="please rename the label")
                    else:
                        mq = MQSend()
                        me = mq.message("MarkNode", label=data["label"], robot_id=robot.id, no_ack=False)
                        res = mq.send(message=me)
                        return self.__response(response_arg=res)
                else:
                    return self.__response(status=1002, response_text="argument error")
            else:
                return self.__response(status=2001, response_text="robot not exist")
        else:
            return self.__response(status=1002, response_text="argument error")

    def execute_mission(self):
        required_arg = ["category", "robotId"]
        data = self.post
        for i in required_arg:
            if i not in data:
                return self.__response(status=1002, response_text="argument error: missing %s" % i)
        category = data["category"]
        temp = Robot.objects.filter(id=data["robotId"].upper())
        if not temp:
            return self.__response(status=2001, response_text="robot not exist")
        robot = temp[0]
        if category == "GoalMove":
            nodes = robot.marknode_set.filter(id=int(data["nodeId"]), status=True)
            if nodes:
                node = nodes[0]
                message = {"x": node.x, "y": node.y, "z": node.rz}
                mq = MQSend()
                me = mq.message("GoalMove", no_ack=False, **message)
                mq.send(queue="BaseMission", message=me)
                # robot.on_mission = True
                # robot.save()
            else:
                self.__response(status=2011, response_text="node %s not existed" % data["nodeId"])
        elif category == "GoalBack":
            message = {"x": 0, "y": 0, "z": 0}
            mq = MQSend()
            me = mq.message("GoalMove", no_ack=False, **message)
            mq.send(queue="BaseMission", message=me)
        # MQSend().send(queue="BaseMission", message=json.dumps(message))
        self.__response()

    def cancel_mission(self):
        mq = MQSend()
        me = mq.message("cancel_all_goal", no_ack=False)
        mq.send(queue="BaseMission", message=me)
        self.__response()

    def clear_costmaps(self):
        mq = MQSend()
        me = mq.message("clear_costmaps", no_ack=False)
        mq.send(queue="BaseMission", message=me)
        self.__response()

    def __response(self, status=0, response_text=None, response_arg=None, **kwargs):
        if not response_text:
            if not status:
                response_text = u"ACCESS SUCCEED"
            else:
                response_text = u"ACCESS FAILED"
        res = {
            "status": status,
            "responseText": response_text,
               }
        if response_arg:
            res["responseArg"] = response_arg
        res.update(**kwargs)
        print(res)
        self.handler.jsonresponse(res)
