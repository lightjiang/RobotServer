#!/usr/bin/python
# Copyright 2015-2016 Fetch Robotics, Inc.
# Author(s): Michael Hwang

## @file rosout_logger.py Class for converting rosout logs to database log
#  entries

# Standard Library
import requests

# Third Party
import rospy
import rosservice
from rosgraph_msgs.msg import Log
from std_msgs.msg import String as StringMessage

## Map of Python logging level strings to ROS Log levels
LOG_LEVELS = {'CRITICAL': Log.FATAL,
              'ERROR': Log.ERROR,
              'WARNING': Log.WARN,
              'INFO': Log.INFO,
              'DEBUG': Log.DEBUG}

TEXT_LOG_LEVELS = {Log.FATAL: 'FATAL',
                   Log.ERROR: 'ERROR',
                   Log.WARN: 'WARN',
                   Log.INFO: 'INFO',
                   Log.DEBUG: 'DEBUG'}

## @brief RosoutLogger subscribes to `/rosout`, parses and filters logs
#  it receives, then uploads them to the database via the server.
class RosoutLogger(object):

    ## @param client The client that the Logger should post logs through
    def __init__(self, client, debug=False):
        self.debug = debug
        self.client = client
        self.allowed_log_level = None
        self.current_task = None
        self.current_goal = None
        self.rosout_subscriber = None
        self.task_subscriber = rospy.Subscriber("~current_task_id",
                                                StringMessage,
                                                self.parse_task)
        self.get_params_from_server()

    def get_params_from_server(self):
        log_level_str = self.get_log_level_str()
        self.allowed_log_level = (LOG_LEVELS[log_level_str] if log_level_str
                                  else Log.WARN)
        log_blacklist = self.get_log_blacklist()
        self.blacklist = log_blacklist if log_blacklist else []

        # subscribers
        self.rosout_subscriber = rospy.Subscriber(
            "/rosout_agg", Log, self.parse_log)

    def get_log_level_str(self):
        # Retrieve log level
        try:
            log_level_str = self.client.get_setting_for_component(
                'logger', 'log_level')
        except requests.exceptions.HTTPError:
            # In case one isn't found, set it as None
            log_level_str = None
        return log_level_str

    def get_log_blacklist(self):
        # Retrieve log blacklist
        try:
            log_blacklist = self.client.get_setting_for_component(
                'logger', 'blacklist')
        except requests.exceptions.HTTPError:
            # In case one isn't found set it as None
            log_blacklist = None
        # self.blacklist needs to be a list, or function will throw error
        if log_blacklist and not isinstance(log_blacklist, list):
            log_blacklist = [log_blacklist]
        return log_blacklist

    ## @param level The python logging level to set the filter level to.
    def set_logger_level(self, level):
        level = level.upper()
        if (level in LOG_LEVELS):
            self.allowed_log_level = LOG_LEVELS[level]
        else:
            rospy.logwarn("Was told to set logger level to unrecognized %s.",
                          level)

    ## Unused, but might be useful in the future.
    def set_rosout_logger_level(self, level):
        level = level.upper()
        if level == "SYSTEM":
            log_level_str = self.get_log_level_str()
            level = log_level_str if log_level_str else level

        if level == "CRITICAL":
            level = "FATAL"

        # This assumes the set_logger_level service runs off of strings, but
        # that's unconfirmed
        servicename = rospy.get_name() + '/set_logger_level'
        service = rosservice.get_service_class_by_name(servicename)
        proxy = rospy.ServiceProxy(str(servicename), service)
        proxy('ros', level)

        # TODO: Potentially set logger level here.

    ## @brief Subscriber callback for storing task ID
    def parse_task(self, msg):
        self.current_task = msg.data if msg.data else None

    ## @brief Subscriber callback for recording rosout messages.
    #  Converts rosout messages into logs on the database.
    def parse_log(self, log):
        # rosgraph fields are:
        # header, level, name, msg, file, function, line, topics
        if (log.level >= self.allowed_log_level and
                log.name not in self.blacklist):

            # TEMPORARY STOPGAP: Remove /move_base logs from being added
            if log.name in ['/move_base', '/follow_pick', '/duckling_mode']:
                # Return early to skip processing
                return

            new_log = {'log_level': TEXT_LOG_LEVELS[log.level],
                       'name': log.name,
                       'file': log.file,
                       'function': log.function,
                       'line': log.line,
                       'message': log.msg,
                       'task': self.current_task}
                       # Timestamp not in correct format, server will auto-assign}
            try:
                self.client.create_log(**new_log)
            except requests.exceptions.HTTPError as e:
                if self.debug:
                    raise e
