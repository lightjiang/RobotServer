# coding=utf-8
from __future__ import unicode_literals

from django.db import models
from core.tools.random import random_str
# Create your models here.


class DataBase(models.Model):
    change_time = models.DateTimeField(auto_now=True)
    change_code = models.CharField(max_length=32, default="")
    create_time = models.DateTimeField(auto_now_add=True)

    class Meta:
        abstract = True

    def my_save(self):
        self.save()
        HistoryData().my_save(self)


class HistoryData(models.Model):
    code = models.CharField(max_length=32, unique=True)
    data_id = models.IntegerField(default=1)
    label = models.CharField(max_length=100, default="")
    content = models.TextField(default="")
    last_code = models.CharField(max_length=32, default="")
    create_time = models.DateTimeField(auto_now_add=True)

    def __unicode__(self):
        return self.code

    def my_save(self, obj):  # 自动记录数据历史，须在保存后调用
        if self.check_change(obj):
            self.code = self.get_code()
            self.data_id = obj.id
            self.label = obj.__class__
            self.content = self.get_value(obj)
            self.save()
            obj.change_code = self.code
            obj.save()
            return 1
        else:
            return 0

    def check_change(self, obj):
        try:
            if obj.change_code == u"":
                self.last_code = ""
                return 1
            else:
                temp = HistoryData.objects.filter(code=obj.change_code)
                if temp:
                    historydata = temp[0]
                    self.last_code = historydata.code
                    content = eval(historydata.content)
                    for i in obj._meta._forward_fields_map.keys():
                        if i not in ["creat_time", "change_time", "change_code"]:
                            if content[i] != unicode(getattr(obj, i)):
                                return 1
                    return 0
                else:
                    self.last_code = ""
                    return 1
        except:

            return 0

    @staticmethod
    def get_value(obj):
        content = {}
        for i in obj._meta._forward_fields_map.keys():
            content[i] = unicode(getattr(obj, i))
        return unicode(content)

    @staticmethod
    def get_code():
        code = random_str(32)
        while HistoryData.objects.filter(code=code):
            code = random_str(32)
        return code


class AccessTrack(models.Model):
    """
    访问轨迹跟踪！！
    """
    active_position = models.CharField(max_length=200)
    active_note = models.CharField(max_length=50)
    active_ip = models.CharField(max_length=40)
    active_datetime = models.DateTimeField(auto_now_add=True)

    def __unicode__(self):
        return u'%s login %s in %s.' % (self.active_ip, self.active_position, self.active_datetime)

    def my_save(self):
        self.save()


class Robot(DataBase):
    id = models.CharField(primary_key=True, max_length=10)
    name = models.CharField(max_length=32, default="")
    icon = models.CharField(max_length=200, default="")
    on_mission = models.BooleanField(default=False)
    charge = models.IntegerField(default=100)

    def __unicode__(self):
        return self.id


class MarkNode(DataBase):
    robot = models.ForeignKey(Robot)
    label = models.CharField(max_length=50, default="")
    status = models.BooleanField(default=True)
    x = models.FloatField(default=0.0)
    y = models.FloatField(default=0.0)
    rz = models.FloatField(default=0.0)
    is_shown = models.BooleanField(default=True)

    def __unicode__(self):
        return "Robot %s's marked_node: %s" % (self.robot.id, self.label)


class Mission(DataBase):
    robot = models.ForeignKey(Robot)
    category = models.CharField(default="", max_length=20)
    goal = models.CharField(max_length=100, default="{'x':0.0, 'y':0.0, 'rz': 0.0}")
    is_delay = models.BooleanField(default=False)
    delay_time = models.IntegerField(default=0)
    is_finished = models.BooleanField(default=False)
    evaluate = models.IntegerField(default=0)
