#!/usr/bin/env python

# Copyright 2016 Fetch Robotics Inc.
# Author: Michael Hwang

## @file dispatch.py Classes for the Dispatch class and its built-in
#  variations.

# Standard Library
import json
import math
import os
import requests
import subprocess
import sys
import uuid
from distutils.version import StrictVersion
from datetime import datetime

# Third Party
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus as gs
from actionlib_msgs.msg import GoalStatusArray
from check_poses.msg import (CheckPosesAction,
                             CheckPosesGoal)
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from tf import (ConnectivityException,
                ExtrapolationException,
                LookupException)

# Fetch
from fetchcore_client.helper_methods import (attr_in_object,
                                             collapse_dictionary,
                                             get_theta,
                                             rgetattr,
                                             rsetattr,
                                             string_to_pcvs,
                                             string_to_posestamped,
                                             xytheta_to_pose)
from fetchcore_client.monitor import Monitor
from fetchcore_client.settings import TIMESTAMP_FORMAT

LIGHT_ON_FIELD = "light_on_at_start"
LIGHT_OFF_FIELD = "light_off_at_end"

## @name goal_states
#  Possible states a ROS action can be in.
## @{
## Enum structure mapping goalstate numbers to readable statuses
GOALSTATES = {gs.PENDING:       'PENDING',
              gs.ACTIVE:        'ACTIVE',
              gs.PREEMPTED:     'PREEMPTED',
              gs.SUCCEEDED:     'SUCCEEDED',
              gs.ABORTED:       'ABORTED',
              gs.REJECTED:      'REJECTED',
              gs.PREEMPTING:    'PREEMPTING',
              gs.RECALLING:     'RECALLING',
              gs.RECALLED:      'RECALLED',
              gs.LOST:          'LOST'}
FAILED_GOALSTATES = [gs.ABORTED,
                     gs.LOST,
                     gs.PREEMPTED,
                     gs.RECALLED,
                     gs.REJECTED]
## @}

## @name task_states
#  Strings matching what states a task or task-action could be.
## @{
TASK_STATE_FAILURE = "FAILED"
TASK_STATE_SUCCESS = "COMPLETE"
TASK_STATE_CANCEL = "CANCELED"
TASK_STATE_PREEMPT = "PREEMPTED"
TASK_STATE_ACTIVE = "WORKING"
TASK_STATE_PENDING = "QUEUED"
## @}

## Abstraction of client-related logic for ROS Actions.
#  Customized actions should extend and/or re-implement this class.
class Dispatch(object):

    ## Initialization method. Can be easily initialized by sending an
    #  unwrapped action definition. The documented params must be imported.
    #  See the task-action schema for definition.
    ## @param action_msg The ROS Action message for client subscription
    ## @param goal_msg The ROS Action Goal message, used for goal construction
    ## @param result_msg The ROS Action Result message.
    def __init__(self,
                 reactor=None,
                 monitor=None,
                 action_msg=None,
                 action_name=None,
                 action_ns=None,
                 action_type=None,
                 goal_msg=None,
                 optional_fields=None,
                 required_actions=None,
                 required_fields=None,
                 required_pkgs=None,
                 result_msg=None,
                 required_config=None,
                 return_fields=None,
                 map_required=True,
                 wait_duration=10,
                 feedback_fields=None,
                 can_timeout=True,
                 **kwargs):

        self.reactor = reactor
        self.monitor = monitor
        self.action_msg = action_msg
        self.goal_msg = goal_msg
        self.name = action_name
        self.ns = action_ns
        self.optional_fields = optional_fields if optional_fields else {}
        self.required_actions = required_actions if required_actions else []
        self.required_fields = required_fields if required_fields else {}
        self.required_pkgs = required_pkgs
        self.result_msg = result_msg
        self.type = action_type
        self.required_config = (required_config if isinstance(required_config,
                                                              list)
                                else [required_config])
        self.return_fields = return_fields if return_fields else {}
        self.return_values = {}
        self.map_required = map_required
        self.wait_duration = wait_duration
        self.feedback_fields = feedback_fields if feedback_fields else {}
        self.client = self.make_client()
        self.last_update_received = None
        self.can_timeout = can_timeout

        self.turn_off_light_on_finish = False

        # For turning lights on when action starts
        self.turn_lights_on_start = False

        if self.ns:
            self.status_sub = rospy.Subscriber(
                self.ns + "/status", GoalStatusArray, self._check_status,
                queue_size=1)

    def _check_status(self, msg):
        # Simple action clients have a single active goal at most
        # If one is active, we are still alive
        for status in msg.status_list:
            if status.status == gs.ACTIVE:
                self.last_update_received = rospy.get_rostime()
                return

    ## Create an action client necessary for operation.
    def make_client(self):
        return actionlib.SimpleActionClient(self.ns, self.action_msg)

    ## Start an associated action with explicit properties.
    ## @param goal A prepopulated goal. This will bypass the create_new_goal()
    #    method if provided.
    ## @param kwargs Key-value pairs that will be used to populate the goal
    ## @returns True if a goal is constructed and sent without error
    ## @returns False otherwise
    def start(self, goal=None, **kwargs):
        # Reset return value
        self.return_values = {}

        # Check reactor state (if reactor provided):
        if self.reactor:
            # are we missing any required actions?
            if self.required_actions:
                required_action_set = set(self.required_actions)
                current_action_set = set(self.reactor.actions.keys())
                actions_needed = required_action_set - current_action_set
                if actions_needed:
                    rospy.logwarn("Cannot execute action %s due to missing "
                                  "actions: %s", self.name,
                                  list(actions_needed))
                    return False

            # are we missing a map when we need one?
            if self.map_required:
                # try to restart the map server if it died
                if self.reactor.map_server and \
                    not self.reactor.map_server.process_alive():
                    rospy.logwarn("Map server dead. Attempting to re-launch.")
                    self.reactor.map_server.launch()
                    # TODO(enhancement): wait for the process to start?

                if not self.reactor.map_server or \
                    not self.reactor.map_server.process_alive():
                    rospy.logwarn("Cannot execute action %s: map required but "
                                  "none available", self.name)
                    return False

            # check if the monitor is working, if we have one
            if self.monitor and not self.monitor.process_alive():
                # We have a monitor but it's not working, so launch it
                rospy.loginfo("Launching the monitor for %s", self.name)
                self.monitor.launch(wait_on_kill=True)
                # Block for set time until the server is available
                available = self.wait_for_server(self.wait_duration)
                if not available:
                    rospy.logwarn("Client did not respond within timeout. "
                                  "(%ss)", self.wait_duration)
                    return False

                # Sometimes wait_for_server isn't good enough
                quarter_wait = self.wait_duration / 4.0
                rospy.loginfo("Waiting for %.2f more seconds for launch...",
                              quarter_wait)
                rospy.sleep(quarter_wait)

            reactor_charging = (self.reactor.is_charging or
                                self.reactor.charger_voltage >= 10)

            # are we charging and MOBILE?
            if "MOBILE" in self.required_config and \
                    reactor_charging and \
                    not self.name in ["LOCALIZE", "UNDOCK"]:
                # if so, send and wait for an undock before doing anything
                undock_dispatch = self.reactor.dispatch.get('UNDOCK')
                rospy.logdebug("Attempting to UNDOCK since we are charging.")
                undock_goal = undock_dispatch.create_new_goal()
                undock_goal.rotate_in_place = True
                goal_started = undock_dispatch.start(undock_goal)
                if goal_started:
                    # goal started, so block until the undock finishes
                    undock_dispatch.client.wait_for_result()

        # Create goal if not specified
        if not goal:
            goal = self.create_new_goal(**kwargs)

        # If unable to create goal, return early
        if not goal:
            if self._goal_required_on_start():
                rospy.logwarn("No goal provided for %s when one was required "
                              "for start.", self.name)
                return False
            else:
                return True

        return self._try_send_goal(goal, **kwargs)

    ## Attempt to apply additional args to goal and then send it
    ## @param goal A goal object to attempt to send
    ## @param kwargs Key-value pairs that will be used to populate the goal
    ## @returns True if a goal is sent
    ## @returns False otherwise
    def _try_send_goal(self, goal, **kwargs):
        # Apply any required or optional fields that haven't been filled yet
        if not self._apply_args(goal, self.required_fields, True, **kwargs):
            return False
        if not self._apply_args(goal, self.optional_fields, False, **kwargs):
            return False

        # Check for special flag for lights turning on if it's an HMI bot
        if (self.reactor and "HMI_BUTTONS" in self.reactor.actions and
                (kwargs.get(LIGHT_ON_FIELD) or self.turn_lights_on_start)):
            # Call service to turn on light
            self.reactor.set_light_state(True)

        # Check for special flag for lights turning off
        if self.reactor and kwargs.get(LIGHT_OFF_FIELD):
            # Call service to turn off light at tend
            self.turn_off_light_on_finish = True

        if self.can_send_goal():

            try:
                self.client.send_goal(goal,
                                      active_cb=self.active_cb,
                                      feedback_cb=self.feedback_cb,
                                      done_cb=self.done_cb)
            except rospy.exceptions.ROSSerializationException:
                rospy.logwarn("Unable to send goal for %s, arguments "
                              "malformed!", self.name)
                return False
            # Set last update received after goal to prevent timeouts
            # from preprocessing delay
            self.last_update_received = rospy.get_rostime()

        else:
            rospy.logwarn("Did not meet conditions to send goal for %s.",
                          self.name)
            return False

        return True

    # Apply fields from kwargs to goal
    def _apply_args(self, goal, fields, required, **kwargs):
        # Apply fields if they haven't been filled yet
        for display_name, field_name in fields.iteritems():
            if attr_in_object(goal, field_name):
                continue
            elif display_name in kwargs:
                try:
                    if required:
                        rospy.logdebug("Attempting to set %s on goal...",
                                       field_name)
                    rsetattr(goal, field_name, kwargs[display_name])
                    if required:
                        rospy.logdebug("...DONE")
                except AttributeError:
                    if required:
                        if not goal:
                            # Now we realize that the goal wasn't made
                            rospy.logerr("Failed to configure proper goal.")
                        else:
                            rospy.logerr("Could not set field %s for goal "
                                         "message of type %s!",
                                         field_name, type(goal))
                        return False
                    else:
                        rospy.logwarn(
                            "Ran into error when trying to set optional field "
                            "%s for %s object: %s", field_name, type(goal), e)
            elif required:
                rospy.logerr("Required data %s not provided in %s action!",
                             display_name, self.name)
                return False

        return True

    # Returns whether or not this dispatch should fail on start if there is
    # no goal. This allows child dispatch implementations to start tasks
    # without actually sending goals
    def _goal_required_on_start(self):
        return True

    ## Catch-all method for checking certain conditions before allowing a goal
    #  to be sent.
    def can_send_goal(self):
        return True

    ## Called when the task transitions to active state. Defunct but nice
    #  to have.
    def active_cb(self):
        pass

    ## Called when receiving a feedback message from the ActionServer.
    #  Updates the last_update_received member by default.
    #  Also updates feedback fields as it receives them
    ## @param feedback_msg The message received on the feedback topic.
    def feedback_cb(self, feedback_msg):
        self.last_update_received = rospy.get_rostime()
        feedback_values = {}
        for field_name, display_name in self.feedback_fields.iteritems():
            try:
                feedback_values[display_name] = rgetattr(feedback_msg,
                                                         field_name)
            except AttributeError as e:
                rospy.logwarn(
                    "Ran into error trying to get feedback field %s for %s "
                    "object: %s", field_name, type(feedback_msg), e)
        # Update the feedback values in the agent state dict
        self.reactor.action_feedback.update(feedback_values)

    ## Called when the task transitions to Done.
    ## @param An actionlib_msgs/GoalStatus (integer) representing the end state
    ## @param result_msg The message received on the result topic.
    def done_cb(self, terminal_state, result_msg):
        # Set last_update_received to None since we don't want to fail later
        # tasks
        self.last_update_received = None

        # Call the reactor's set_light_state() to turn off a light when
        # finishing an action via actionlib.
        if self.turn_off_light_on_finish and self.reactor:
            self.turn_off_light_on_finish = False
            self.reactor.set_light_state(False)

        # Clear the feedback fields
        self.reactor.action_feedback = {}

        # Populate the return values
        for field_name, display_name in self.return_fields.iteritems():
            if display_name not in self.return_values:
                # Only add field if it hasn't been updated manually
                value = None
                try:
                    value = rgetattr(result_msg, field_name)
                except AttributeError as e:
                    rospy.logwarn(
                        "Ran into error when trying to get return field %s "
                        "for %s object: %s", field_name, type(result_msg), e)
                if value:
                    if hasattr(value, 'to_time'):
                        value = value.to_time()
                    elif hasattr(value, 'to_sec'):
                        value = value.to_sec()
                    self.return_values[display_name] = value

    ## Creates a new goal instance from the class specified in self.goal_msg.
    #  Mainly used for preprocessing goals fields that required data types
    #  not traditionally supported by JSON.
    #  Should be extended in child classes, if necessary.
    def create_new_goal(self, **kwargs):
        goal = self.goal_msg()
        return goal

    ## @returns String representing the status of the action
    def get_status(self):
        current_goalstate = self.client.get_state()
        if current_goalstate in FAILED_GOALSTATES:
            # failed the action
            return TASK_STATE_FAILURE
        elif current_goalstate == gs.SUCCEEDED:
            return TASK_STATE_SUCCESS
        else:
            return TASK_STATE_ACTIVE

    ## @name actionclient_wrappers
    #  Methods that mimic the ActionClient interface, mostly for
    #  syntactic sugar.
    ## Cancel all the goals for the specific client
    def cancel_all_goals(self):

        # Call the reactor's set_light_state() to turn off a light when
        # finishing an action via canceling or cleanup. This will be called
        if self.turn_off_light_on_finish and self.reactor:
            self.turn_off_light_on_finish = False
            self.reactor.set_light_state(False)

        self.client.cancel_all_goals()

    ## Wait for the ActionServer to respond.
    ## @param duration Can be a number or a rospy Duration. Not providing
    #  this argument will have the client wait indefinitely, as per normal
    #  SimpleActionClient behavior.
    ## @return True if the server responded before timeout
    ## @return False otherwise
    def wait_for_server(self, duration=0):
        if not duration:
            return self.client.wait_for_server()
        else:
            if not isinstance(duration, rospy.Duration):
                duration = rospy.Duration(duration)
            return self.client.wait_for_server(duration)

## Implementation of Dispatch class for NAVIGATE goals.
class DispatchNavigate(Dispatch):

    def __init__(self, reactor, **kwargs):
        super(DispatchNavigate, self).__init__(reactor=reactor, **kwargs)
        self.reactor = reactor

        # The timestamp from the last time the path was updated
        self.path_creation_timestamp = None
        # The timestamp when we started waiting for a dynamic path
        self.path_wait_timestamp = None
        # Keep the action key-value pairs passed into start/create_new_goal
        self.current_action_data = None

        self.turn_lights_on_start = True

    # Maximum time (in seconds) to wait for a dynamic path on the paths API
    def get_max_path_wait_time(self):
        try:
            return float(rospy.get_param("~max_path_wait_time", 10.0))
        except ValueError:
            return 10.0

    def create_new_goal(self, **kwargs):
        # Attempting to turn off lights regardless of HMI config is fine, so
        # always set it on start.
        self.turn_off_light_on_finish = True

        # Cache the action data itself
        self.current_action_data = kwargs

        # Is this going to be a dynamic path?
        if 'path_uuid' in kwargs:
            # Don't actually create a goal, we'll do that when we read the path
            # from the paths API. Just record when the action was started.
            self.path_wait_timestamp = \
                int((datetime.utcnow() -
                     datetime.utcfromtimestamp(0)).total_seconds())
            # This is a new action and we haven't gotten the path yet
            self.path_creation_timestamp = None
        else:
            # Not using dynamic paths
            self.path_wait_timestamp = None
            self.path_creation_timestamp = None
            try:
                path = self.current_action_data['target_poses']
            except KeyError:
                rospy.logwarn("Required field 'target_poses' not provided for "
                              "NAVIGATE action!")
                return None

            goal_poses = self._string_path_to_posestamped_path(path)
            if not goal_poses:
                return None

            goal = self.goal_msg()
            goal.target_poses = goal_poses

            self.add_collision_params_to_goal(goal, **kwargs)

            return goal

        return None

    ## Adds collision parameters in reactor to goal if they exist. Modifies
    #  **goal** in-place.
    def add_collision_params_to_goal(self, goal, **kwargs):
        # Update the goal here; assume that update_collision_model either
        # missing or False means that there are no collision parameters in
        # the action
        if (not kwargs.get("update_collision_model") and
                self.reactor.collision_params):
            try:
                setattr(goal, "update_collision_model", True)
                flat_dict = collapse_dictionary(
                    {"collision_model": self.reactor.collision_params})
                for key, val in flat_dict.iteritems():
                    rsetattr(goal, key, val)
                    self.current_action_data[key] = val
            except AttributeError as e:
                rospy.logwarn(
                    "There was an error when applying configuration-"
                    "specific collision parameters for a %s action: %s",
                    self.name, e.message)
                setattr(goal, "update_collision_model", False)

    # Convert a string path (like those returned in document or paths API)
    # into a posestamped path
    def _string_path_to_posestamped_path(self, path):
        poses = []

        try:
            # Now extract the path poses and push them onto our goal
            for idx, strpose in enumerate(path):
                n_pose = string_to_posestamped(strpose, idx)
                poses.append(n_pose)
        except TypeError as e:
            rospy.logwarn(e)
            rospy.logwarn("Invalid path object. Cannot convert poses.")
            return None
        except (ValueError, AttributeError) as e:
            rospy.logwarn(e)
            rospy.logwarn("Cannot extract pose from: " + str(strpose))
            return None

        return poses

    # Try to pull a dynamic path down from the paths endpoint
    # Return a tuple of the path itself and its creation timestamp
    def _get_dynamic_path(self, **kwargs):
        # If there is a path ID, look it up through the paths API and use that
        path = None
        timestamp = None
        if 'path_uuid' in kwargs:
            path_uuid = str(kwargs['path_uuid'])
            try:
                url = self.reactor.client.apiurl + "paths/" + path_uuid
                path_resp = self.reactor.client.get_document(url)
                path = path_resp.get('target_poses')
                timestamp = path_resp.get('timestamp')

            except requests.exceptions.HTTPError as e:
                rospy.logwarn("Unable to GET %s" % url)
            except AttributeError as e:
                rospy.logwarn("Malformed path data. "
                              "Cannot use paths API.")
        else:
            rospy.logdebug("No UUID, not using paths API")
            return (None, None)

        # Only return the data if both the path and timestamp exist
        if not path or not timestamp:
            rospy.logwarn("Path or timestamp missing in returned data. "
                          "Not using paths API")
            return (None, None)

        return (path, timestamp)

    ## @return True if map_server instance is initialized and client is up
    ## @return False otherwise
    def can_send_goal(self):
        if not bool(self.reactor.localized):
            rospy.logwarn("Cannot start NAVIGATE, robot not localized")
            return False
        if not bool(self.reactor.map_server):
            rospy.logwarn("Cannot start NAVIGATE, robot has no map")
            return False
        if not self.client.wait_for_server(rospy.Duration(10)):
            rospy.logwarn("Cannot start NAVIGATE, "
                          "action server did not respond")
            return False
        return True

    ## @returns String representing the status of the action
    def get_status(self):
        # If we aren't localized, terminate now instead of waiting for the
        # dynamic path and failing then
        if not self.reactor.localized:
            rospy.logwarn("Robot not localized. Cannot navigate.")
            return TASK_STATE_FAILURE

        # If we have a path_wait_timestamp, the action has started but we have
        # not retrieved a path from the paths API yet.
        if self.path_wait_timestamp:
            # Check if we've been waiting for too long
            current_timestamp = \
                int((datetime.utcnow() -
                     datetime.utcfromtimestamp(0)).total_seconds())

            if (current_timestamp - self.path_wait_timestamp >
                    self.get_max_path_wait_time()):
                # Give up and use the static path
                rospy.logwarn("Gave up waiting for path from paths API. "
                              "Reverting to static path in document.")
                self.path_wait_timestamp = None

                try:
                    path = self.current_action_data['target_poses']
                except KeyError:
                    rospy.logwarn("Required field 'target_poses' not provided for "
                                  "NAVIGATE action!")
                    return TASK_STATE_FAILURE

                goal_poses = self._string_path_to_posestamped_path(path)
                if not goal_poses:
                    return TASK_STATE_FAILURE

                goal = self.goal_msg()
                goal.target_poses = goal_poses
                self.add_collision_params_to_goal(
                    goal, **self.current_action_data)
                self._try_send_goal(goal, **self.current_action_data)

        # If we are waiting for a dynamic path or already have one,
        # check for a new one.
        if self.path_wait_timestamp or self.path_creation_timestamp:
            # Check the paths API and send down a goal if we got one
            path, timestamp = self._get_dynamic_path(**self.current_action_data)

            if timestamp > self.path_creation_timestamp:
                self.path_creation_timestamp = timestamp
                self.path_wait_timestamp = None

                goal_poses = self._string_path_to_posestamped_path(path)

                if not goal_poses:
                    rospy.logerr("Invalid dynamic path received.")
                    return TASK_STATE_FAILURE

                # Try to send down the updated goal
                goal = self.goal_msg()
                goal.target_poses = goal_poses
                self.add_collision_params_to_goal(goal)
                self._try_send_goal(goal, **self.current_action_data)

        # If we are still waiting for a path, bypass client and report active
        if self.path_wait_timestamp:
            return TASK_STATE_ACTIVE

        current_goalstate = self.client.get_state()
        if current_goalstate in FAILED_GOALSTATES:
            # failed the action
            return TASK_STATE_FAILURE
        elif current_goalstate == gs.SUCCEEDED:
            return TASK_STATE_SUCCESS
        else:
            return TASK_STATE_ACTIVE

    # Don't require goals on start so that we can recieve paths dynamically
    # after navigate tasks are started
    def _goal_required_on_start(self):
        return not bool(self.path_wait_timestamp)


class DispatchMonitoredNavigate(DispatchNavigate):
    event_types = {
        1: "planner",
        2: "controller",
        3: "recovery"
    }

    def done_cb(self, terminal_state, result_msg):
        super(DispatchMonitoredNavigate, self).done_cb(
            terminal_state, result_msg)
        for event_type in self.event_types.values():
            self.return_values[event_type + '_events'] = 0
        for event in result_msg.statistics.events:
            field_name = self.event_types[event.type] + '_events'
            self.return_values[field_name] += 1


## @name waitfor
#  Classes and methods related to WAITFOR actions.
## @{

## Implementation of Dispatch class for WAITFOR actions.
class DispatchWaitfor(Dispatch):

    def __init__(self, **kwargs):
        super(DispatchWaitfor, self).__init__(**kwargs)

    def make_client(self):
        return WaitforClient()

    def create_new_goal(self, **kwargs):
        return WaitforGoal()

    def wait_for_server(self, duration=0):
        return True

## Simple class representing waitfor goal (to work with attribute assignment).
#  Implemented using rostime
class WaitforGoal(object):

    ## @param duration The number of seconds to wait.
    def __init__(self, duration=0):
        self._duration = None
        self.duration = duration

    @property
    def duration(self):
        return self._duration

    @duration.setter
    def duration(self, value):
        if isinstance(value, rospy.Duration):
            self._duration = value
        else:
            self._duration = rospy.Duration(value)

## Class that partially mimicks the ActionClient interface for waitfor actions.
#  Does not support queueing actions.
class WaitforClient(object):

    def __init__(self):
        self.end_rostime = None

    ## @brief Goals will be bound between [0, now() - maxint]
    def send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        # Freights can't time travel (yet), we will make negative times essentially 0
        if goal.duration < rospy.Duration(0):
            duration = rospy.Duration(0)
        else:
            duration = goal.duration

        # Prevent an integer overflow as well
        now = rospy.get_rostime()
        max_time = rospy.Time(sys.maxint)
        if now > max_time - duration:
            self.end_rostime = max_time
        else:
            self.end_rostime = now + duration

    def cancel_all_goals(self):
        self.end_rostime = None

    def get_state(self):
        if not self.end_rostime:
            rospy.logerr("Called get_state() when no goal is running")
            return gs.LOST
        elif rospy.get_rostime() >= self.end_rostime:
            return gs.SUCCEEDED
        else:
            return gs.ACTIVE

## @}

## @name localize
#  Classes and methods related to the LOCALIZE action.
## @{

class DispatchLocalize(Dispatch):
    def __init__(self, reactor, **kwargs):
        super(DispatchLocalize, self).__init__(reactor=reactor, **kwargs)
        self.reactor = reactor
        self.check_pose_client = None

        # Some tunable parameters for localization checking
        self.amcl_weight_high = rospy.get_param('/check_poses/weight_high', 11)
        self.amcl_weight_low = rospy.get_param('/check_poses/weight_low', 2)
        self.amcl_weight_tolerance = \
            rospy.get_param('/check_poses/weight_tolerance', 0.85)

    @property
    def last_update_received(self):
        return self.client.last_update_received

    @last_update_received.setter
    def last_update_received(self, value):
        self.client.last_update_received = value

    def make_client(self):
        return LocalizeClient(self.reactor)

    def create_new_goal(self, **kwargs):
        goal = LocalizeGoal()
        try:
            t_pose = kwargs['target_pose']
            goal.pose = string_to_pcvs(t_pose)
        except KeyError, e:
            rospy.logwarn(e)
            rospy.logwarn("Required field 'target_pose' not provided for "
                          "LOCALIZE action!")
            return None

        # set frame ID in header the same as currently set by reactor
        goal.pose.header.frame_id = self.reactor.map_frame

        return goal

    def reset_check_pose_client(self):
        if self.check_pose_client:
            self.check_pose_client.action_client.cancel_all_goals()
            # TODO(bug): Need to clean up/dealloc, but SimpleActionClient
            #            currently has no destructor or equivalent.

        self.check_pose_client = \
            actionlib.SimpleActionClient('check_poses/check_poses',
                                         CheckPosesAction)

    def get_amcl_weight(self, x, y, theta, timeout=0.05):

        if not self.check_pose_client:
            return None

        if not self.check_pose_client.wait_for_server(
                rospy.Duration.from_sec(timeout)):
            return None

        pose = xytheta_to_pose(x, y, theta)
        goal = CheckPosesGoal()
        goal.poses.poses.append(pose)
        goal.poses.header.stamp = rospy.Time.now()
        goal.poses.header.frame_id = "map"

        self.check_pose_client.send_goal(goal)

        if not self.check_pose_client.wait_for_result(
                rospy.Duration.from_sec(timeout)):
            return None

        result = self.check_pose_client.get_result()
        if not result or not result.is_valid or not result.weights:
            return None
        return result.weights[0]

    # If the robot is not localized, try the last localized position in the
    # agent doc and see if it's still valid and sane. If so, set localized.
    def check_old_localization(self, agent_doc, localization_timeout,
                               amcl_timeout, on_start=False):
        rospy.loginfo("Checking previous robot localization for validity.")

        # Grab the data related to our previous localization from the agent doc
        try:
            agent_state = agent_doc.get('agent_state')
            was_localized = agent_state.get('localized')
            pose_str = agent_state.get('current_pose')
            timestamp = agent_doc.get('last_updated_timestamp')
            amcl_weight = agent_state.get('amcl_weight')
        except AttributeError:
            rospy.logerr("Agent document malformed. "
                         "Cannot check localization.")
            return False

        if not was_localized:
            rospy.loginfo("Robot was not localized when it was shut down. "
                          "No previous localization to check.")
            return False

        # If the last time the robot reported being localized was too long ago,
        # don't try to re-use the old pose
        try:
            localized_time = datetime.strptime(timestamp, TIMESTAMP_FORMAT)
        except (ValueError, TypeError):
            rospy.logerr("Unable to extract time from \"%s\". "
                         "Cannot check localization.", str(timestamp))
            return False

        current_time = datetime.utcnow()
        time_since_localized = (current_time - localized_time).total_seconds()
        if time_since_localized > localization_timeout:
            rospy.loginfo("Previous localized data is too old (%d seconds). "
                          "Not checking localization.", time_since_localized)
            return False

        rospy.loginfo("Was last localized %d seconds ago",
                      time_since_localized)

        try:
            x, y, theta = [float(f) for f in pose_str.split(",")]
        except (ValueError, AttributeError):
            rospy.logerr("Unable to get pose from \"%s\". "
                         "Cannot check localization.", str(pose_str))
            return False

        # Check AMCL for the last pose the robot reported before going down,
        # which may be slightly newer than the one in the doc (above).
        try:
            amcl_x = rospy.get_param("/amcl/initial_pose_x")
            amcl_y = rospy.get_param("/amcl/initial_pose_y")
            amcl_a = rospy.get_param("/amcl/initial_pose_a")

            # If pose is (0,0,0), AMCL just initialized so the pose is invalid.
            # NOTE: This relies on the fact that (0,0) is always off our maps!
            thresh = 0.1
            if abs(x) > thresh or abs(y) > thresh or abs(theta) > thresh:
                x = amcl_x
                y = amcl_y
                theta = amcl_a
        except KeyError:
            rospy.logdebug("AMCL not even up yet. Robot must have just "
                           "started up. No latest AMCL pose to test.")

        # This check doesn't happen often and may happen right after the map
        # server comes up, so allow a longer timeout
        weight = self.get_amcl_weight(x, y, theta, amcl_timeout)

        rospy.loginfo("Previous AMCL weight: %s, Current AMCL weight: %s",
                      str(amcl_weight), str(weight))

        # If the weight is high enough, call it good. If it's low enough,
        # it's no good, and otherwise make sure it's higher than some
        # percentage of the old weight (if there is one)
        if weight > self.amcl_weight_high or \
            (amcl_weight and
             weight > self.amcl_weight_low and
             weight > amcl_weight * self.amcl_weight_tolerance):

            pstring = "%.2f,%.2f,%.4f" % (x, y, theta)
            rospy.loginfo("New weight is sufficient. Marking localized at %s",
                          pstring)

            # Use dispatch to send down localization goal
            self.client.set_on_start(on_start)
            self.start(target_pose=pstring)

            # Wait up to 10 seconds for the localization to complete
            poll_start = rospy.get_time()
            while rospy.get_time() - poll_start < 10:
                status = self.get_status()
                if status == TASK_STATE_SUCCESS:
                    return True
                rospy.sleep(0.05)

            rospy.loginfo("Timed out waiting for localization to complete.")

        return False

class LocalizeGoal(object):
    def __init__(self, pose=None):
        self.pose = pose

class LocalizeClient(object):

    def __init__(self, reactor):

        # Goal will be stored, and then checked against amcl pose value
        self.goal = None
        self.local_pub = None
        self.local_sub = None
        self.state = gs.LOST
        self.reactor = reactor
        self.last_update_received = None
        ## Is the client being called when the reactor starts
        self.is_on_start = False

        self.clear_costmap_service = rospy.ServiceProxy(
            '/move_base/clear_costmaps',
            Empty)
        # Number of seconds to wait on second attempt to clear costmaps
        self.ccs_delay = 2.0

    def __del__(self):

        if self.local_sub:
            self.local_sub.unregister()
        if self.local_pub:
            self.local_pub.unregister()

    def set_on_start(self, on_start):
        self.is_on_start = on_start

    def process_localization_state(self, msg):

        if not self.goal:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        th = get_theta(msg.pose.pose)

        x_err = math.fabs(x - self.goal.pose.pose.position.x)
        y_err = math.fabs(y - self.goal.pose.pose.position.y)
        th_err = math.fabs(th - get_theta(self.goal.pose.pose))
        if th_err > math.pi:
            th_err = math.fabs(th_err - math.pi * 2.0)

        if x_err < 0.1 and y_err < 0.1 and th_err < 0.25:

            # Tell the reactor that it should be localized now
            self.reactor.localized = True

            # Wrap up localization
            self.state = gs.SUCCEEDED
            self.cancel_all_goals()

    def wait_for_server(self, duration=0):
        return True

    ## Clear costmaps by calling the service in self.clear_costmap_service.
    ## @returns True if the costmaps were cleared without error
    ## @returns False otherwise
    def clear_costmap(self):
        try:
            self.clear_costmap_service()
        except rospy.ServiceException:
            nav_disp = self.reactor.dispatch.get("NAVIGATE")
            if (not self.is_on_start) and (nav_disp and nav_disp.monitor and nav_disp.monitor.process_alive()):
                return False
        return True

    def cancel_all_goals(self):

        # Clear costmaps if LOCALIZE succeeded
        if self.state == gs.SUCCEEDED:
            if not self.clear_costmap():
                # We might fail to clear costmaps on the first try, so let's try again after waiting
                rospy.logwarn("Failed to clear costmaps on initial attempt. Waiting %s seconds before clearing again.",
                              self.ccs_delay)
                rospy.sleep(self.ccs_delay)
                if not self.clear_costmap():
                    rospy.logwarn("Failed to clear costmaps on final attempt. Failing LOCALIZE action.")
                    self.state = gs.ABORTED

        self.last_update_received = None

        if self.local_sub:
            try:
                self.local_sub.unregister()
            except AssertionError:
                # We've double-unregistered
                rospy.logwarn("Attempted to unregister localization subscriber"
                              " when subscribed topic does not exist!")
            self.local_sub = None
        if self.local_pub:
            try:
                self.local_pub.unregister()
            except AssertionError:
                # We've double-unregistered
                rospy.logwarn("Attempted to unregister localization publisher "
                              " when published topic does not exist!")
            self.local_pub = None

    def send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):

        # Subscribe to AMCL pose so we can determine when we are localized
        self.local_sub = rospy.Subscriber("/amcl_pose",
                                          PoseWithCovarianceStamped,
                                          self.process_localization_state)

        # Publish our goal when we get one
        self.local_pub = rospy.Publisher("/initialpose",
                                         PoseWithCovarianceStamped,
                                         queue_size=1,
                                         latch=True)

        # Publish our localzation
        self.state = gs.ACTIVE
        self.goal = goal.pose
        self.last_update_received = rospy.get_rostime()

        # If we are re-localizing, we must have been unlocalized
        self.reactor.localized = False

        self.local_pub.publish(self.goal)


    def get_state(self):

        return self.state

## @}

## @name buildmap
#  Classes and methods related to the BUILDMAP action.
## @{

class DispatchBuildmap(Dispatch):
    def __init__(self, reactor, **kwargs):
        # Set ros parameter for saving as PNG
        rospy.set_param("/fetch_map_saver/save_file_type", "png")
        super(DispatchBuildmap, self).__init__(reactor=reactor, **kwargs)
        self.reactor = reactor

    def create_new_goal(self, **kwargs):
        return BuildmapGoal()

    def make_client(self):
        return BuildmapClient(self.reactor)

class BuildmapGoal(object):
    def __init__(self, mapname=None):
        self.mapname = mapname

class BuildmapClient(object):

    def __init__(self, reactor):
        self.reactor = reactor
        self.goal_state = None
        ## The mapname of the new map
        self.mapname = None
        ## The map ID of the new map (generated on start)
        self.map_id_to_save = None
        self._builder = None

        self.last_map_upload = None
        self.already_created = False
        self.last_known_pose = None

    ## Builder as property to handle sending SIGINT on new assignment
    @property
    def builder(self):
        return self._builder

    @builder.setter
    def builder(self, value):
        if self._builder:
            try:
                if isinstance(self._builder, Monitor):
                    # Kill previous instance if it's a subprocess
                    try:
                        rospy.logdebug("Killing the map builder...")
                        self._builder.kill_process(wait=True)
                        rospy.logdebug("...DONE")
                    except OSError:
                        rospy.logerr("Failed to kill non-existent map builder")
            except TypeError:
                # We hit a TypeError only during testing due to mock, so
                # ignore it. Proper mocking behavior will be necessary later.
                pass
        self._builder = value

    def __del__(self):
        # Last-ditch trigger for the builder cleanup
        self._builder = None

    def wait_for_server(self, duration=0):
        return True

    def send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        self.already_created = False

        # Set goal state to ACTIVE
        self.goal_state = gs.ACTIVE

        # Generate new map ID to save as
        self.map_id_to_save = "MAP-" + str(uuid.uuid4())
        # Grab parameters from goal
        self.mapname = goal.mapname

        # Stop map server instance in self.reactor
        self.reactor.map_server = None

        # Start map building instance
        self.builder = Monitor(self,
                               launch_pkg="slam_it",
                               launch_file="build_map.launch")
        self.builder.launch(wait_on_startup=False)

        self.last_map_upload = rospy.get_rostime()
        self.last_known_pose = self.get_current_pose_from_reactor()

    def get_current_pose_from_reactor(self):
        try:
            xyz, _ = self.reactor.tf_listener.lookupTransform(
                self.reactor.map_frame,
                'base_link',
                rospy.Time(0))
            return xyz[:2]
        except (ConnectivityException, ExtrapolationException,
                LookupException):
            # tf hasn't started up yet
            return None

    def cancel_all_goals(self):
        if self.builder:
            # Save the map with map_server using param given in goal
            savepath = self.save_map_to_disk()

            # Wind down the map builder
            self.builder = None

            # Upload to the server
            response = None
            try:
                response = self.upload_map_to_server(savepath)
            except requests.exceptions.HTTPError as e:
                rospy.logerr("BUILDMAP upload for %s failed with error: %s",
                             self.map_id_to_save, e)

            if not response:
                rospy.logerr("Server did not respond to final upload request. "
                             "Map may not have been completed properly.")

            # Start the previous map server
            if self.reactor.map_id:
                self.reactor.launch_map_server()

            # Mark the reactor as delocalized
            self.reactor.localized = False

            # Set goal_state to SUCCEEDED
            # TODO: Determine if this needs to be something else for preempt logic to work.
            self.goal_state = gs.SUCCEEDED

    def save_map_to_disk(self):
        savepath = os.path.join(self.reactor.map_folder, self.map_id_to_save)

        if not os.path.exists(savepath):
            os.makedirs(savepath)

        subprocess.call(['rosrun', 'fetch_map_server', 'fetch_map_saver'],
                        cwd=savepath)

        return savepath

    def upload_map_to_server(self, savepath, in_progress=False):
        if self.already_created:
            response = self.reactor.client.update_map(
                self.map_id_to_save, map_folder=savepath,
                map_image='map.png', map_yaml='map.yaml',
                in_progress=in_progress)
        else:
            response = self.reactor.client.upload_map(
                self.mapname, savepath, in_progress=in_progress,
                map_id=self.map_id_to_save,
                buildmap_task_id=self.reactor.current_task_id)
            self.already_created = bool(response)
        self.last_map_upload = rospy.get_rostime()
        return response

    def process_goal(self):
        if self.builder:
            # Upload the map if we're past a certain time/position threshold,
            # or if we haven't created the map yet.
            if self.can_upload() or not self.already_created:
                savepath = self.save_map_to_disk()
                try:
                    self.upload_map_to_server(savepath, in_progress=True)
                    self.last_known_pose = self.get_current_pose_from_reactor()
                except requests.exceptions.HTTPError as e:
                    rospy.logerr("BUILDMAP incremental upload for %s failed "
                                 "with error: %s", self.map_id_to_save, e)
                    rospy.logwarn("Wrapping up BUILDMAP task.")
                    self.builder = None
                    self.goal_state = gs.ABORTED

    ## @returns True if we've moved more than 10 centimeters and it's 
    ## been more than 5 seconds since the last check.
    def can_upload(self):
        past_time = (rospy.get_rostime() >=
                     (self.last_map_upload + rospy.Duration(5)))
        if self.last_known_pose:
            curr_pose = self.get_current_pose_from_reactor()
            try:
                delta_x = curr_pose[0] - self.last_known_pose[0]
                delta_y = curr_pose[1] - self.last_known_pose[1]
                total_dist = math.sqrt(delta_x**2 + delta_y**2)

                past_prev_location = total_dist > 0.1
            except (TypeError, AttributeError):
                # We didn't get a current pose, so let's assume we aren't
                # past our previous location
                past_prev_location = False
        else:
            # We don't have a last known pose yet
            self.last_known_pose = self.get_current_pose_from_reactor()
            past_prev_location = True
            past_time = True

        return past_time and past_prev_location

    def get_state(self):
        if not self.goal_state:
            rospy.logerr("Called get_state() when no goal is running")
            return gs.LOST
        else:
            self.process_goal()
            return self.goal_state

## @}

## @name update
#  Classes and actions related to the UPDATE action
## @{
class DispatchUpdate(Dispatch):

    def __init__(self, reactor, **kwargs):
        super(DispatchUpdate, self).__init__(reactor=reactor, **kwargs)
        self.reactor = reactor

    def make_client(self):
        return UpdateClient(self.reactor)

    def create_new_goal(self, **kwargs):
        return UpdateGoal()

class UpdateGoal(object):

    def __init__(self):
        self.sources = []
        self.override_sources = False
        self.aptkeys = []

class UpdateClient(object):

    def __init__(self, reactor):
        self.reactor = reactor
        self.goal_state = None
        self.sources_file = rospy.get_param(
            "~sources_file", "/etc/apt/sources.list.d/fetchcore-latest.list")
        self.daemon_pid = None

    def wait_for_server(self, duration=0):
        return True

    def cancel_all_goals(self):
        pass

    ## UPDATE action behavior
    def send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):

        # Set goal state to ACTIVE
        self.goal_state = gs.ACTIVE
        rospy.loginfo("Updating the Robot.")

        # Get our configuration from the server
        self.reactor.update_from_agent_doc()

        # Get action list from server
        available_actions = \
            self.reactor.client.get_available_action_definitions()

        if not available_actions:
            rospy.logerr("Did not receive available action definitions from "
                         "server. Aborting UPDATE.")
            self.goal_state = gs.ABORTED
            return

        # Get action blacklist
        blacklisted_actions = self.reactor.client.get_setting_for_component(
            "update", "action_blacklist")

        # Remove blacklisted actions from known pool of available actions
        if blacklisted_actions:
            for action_name in blacklisted_actions:
                rospy.loginfo("Got blacklisted action %s as available action. "
                              "Removing from available pool.", action_name)
                del available_actions[action_name]

        rospy.loginfo("Deleting previous action definitions stored in %s",
                      self.reactor.actdef_folder)
        # Clean up files in action folder before download
        for action_file in os.listdir(self.reactor.actdef_folder):
            path = os.path.join(self.reactor.actdef_folder, action_file)
            if os.path.isfile(path):
                os.unlink(path)

        # Save available actions
        for action_name in available_actions:
            if self.action_config_valid(available_actions[action_name]):
                rospy.loginfo("Installing %s", action_name)
                if not self.get_and_save_actdef(action_name):
                    # Warning exists already in the conditional method
                    continue
            else:
                rospy.loginfo("Skipping %s (robot not configured to"
                              " support)", action_name)

        # Update sources
        if goal.sources and not self.reactor.source_mode:
            # Single item JSON list comprehension fix
            if not isinstance(goal.sources, list):
                goal.sources = [goal.sources]
            # Update fetchcore_latest.list with entries from Goal fields
            if goal.override_sources:
                read_mode = "w"
            else:
                read_mode = "a"
            # TODO: Check for file existence before opening
            if not os.path.isfile(self.sources_file):
                rospy.logerr("Optional field 'sources' was provided but it "
                             "looks like this robot's sources list (%s) was "
                             "not created beforehand. Aborting UPDATE task.",
                             self.sources_file)
                self.goal_state = gs.ABORTED
                return

            with open(self.sources_file, read_mode) as source_file:
                if read_mode == "a":
                    source_file.write("\n")
                for source_item in goal.sources:
                    rospy.loginfo("Adding %s to source.list", source_item)
                    source_file.write("%s\n" % source_item)

        # Add GPG keys, if given (must be URL format that is downloadable)
        if goal.aptkeys and not self.reactor.test_mode:
            # Single item JSON list comprehension fix
            if not isinstance(goal.sources, list):
                goal.sources = [goal.sources]
            # Make temporary folder to store downloaded keys in
            temp_folder = "/tmp/%s" % uuid.uuid4()
            os.mkdir(temp_folder)
            for key_url in goal.aptkeys:
                # Extract filename from URL
                key_url_split = key_url.split("/")
                key_filename = key_url_split[-1]
                try:
                    resp = self.reactor.client.make_request('GET', key_url)
                except requests.exceptions.HTTPError as e:
                    rospy.logerr("Ran into error when downloading specified "
                                 "apt key from %s: %s. Aborting UPDATE task.",
                                 key_url, e)
                    self.goal_state = gs.ABORTED
                    return
                if not resp:
                    # we did not connect properly
                    rospy.logerr("Expected response but received nothing. "
                                 "Aborting UPDATE task.")
                    self.goal_state = gs.ABORTED
                    return
                # Save the key in the temp folder
                with open(os.path.join(temp_folder, key_filename), 'w') as f:
                    f.write(resp.content)
                # Add the key to the apt-key list
                rospy.loginfo("Adding %s to apt-keys", key_filename)
                apt_key_code = subprocess.call(['sudo', 'apt-key', 'add',
                                                os.path.join(temp_folder,
                                                             key_filename)])
                if apt_key_code != 0:
                    rospy.logwarn("Did not successfully add apt key %s. "
                                  "Exit code: %i", key_filename, apt_key_code)
            # TODO: Determine if we need to delete the GPG keys or not, since they are in a /tmp/ folder now

        # Run apt-get update
        if not self.reactor.source_mode:
            rospy.loginfo("Updating apt repository listings")
            update_code = subprocess.call(['sudo', 'apt-get', 'update'])
            if update_code != 0:
                rospy.logwarn("Could not successfully update apt sources. "
                              "Exit code: %i", update_code)

            rospy.loginfo("Upgrading packages")
            upgrade_code = subprocess.call(['sudo', 'DEBIAN_FRONTEND=noninteractive', 'apt-get', 'upgrade', '-y'])
            if upgrade_code != 0:
                rospy.logwarn("Could not successfully upgrade packages. "
                              "Exit code: %i", upgrade_code)

        # Crawl through actdefs downloaded and install required packages
        self.goal_state = self.install_dependencies()

    def get_state(self):
        if not self.goal_state:
            rospy.logerr("Called get_state() when no goal is running")
            return gs.LOST
        else:
            if self.goal_state == -1:
                # We're installing apt stuff with a daemon, so we need to check
                # if that install process is still going
                if os.path.exists("/proc/%s" % self.daemon_pid):
                    # For POSIX systems, a process is still running if there
                    # is a folder with its PID in the /proc/ directory
                    return gs.ACTIVE
                else:
                    # The process finished, so change the goal state and let
                    # the cleanup code below execute
                    self.daemon_pid = None
                    self.goal_state = gs.SUCCEEDED

            if self.goal_state == gs.SUCCEEDED:
                # Manually set task status to success and restart
                self.reactor._end_current_task('COMPLETE')
                rospy.logwarn("UPDATE was a success. Shutting down reactor"
                              " to apply changes.")
                sys.exit()

        return self.goal_state

    ## @param action_name The name of an action defined on the server.
    ## @returns True if we got some content for **action_name** from the server
    ## @returns False otherwise
    def get_and_save_actdef(self, action_name):
        actdef = self.reactor.client.get_action_definition(action_name)
        # Check to make sure we don't have an empty action definition
        if not actdef:
            rospy.logerr("Received empty action definition for %s from server."
                         " It may be due to network or user error. Skipping "
                         "download for this action.",
                         action_name)
            return False
        with open(
            os.path.join(self.reactor.actdef_folder,
                         "%s.json" % action_name.lower()), "w") as f:
            json.dump(actdef, f, indent=4, sort_keys=True)
        return True

    ## @param basic_actdef A dictionary with a field 'required_config'
    def action_config_valid(self, basic_actdef):
        required_config = set(basic_actdef['required_config'])
        current_config = set(self.reactor.configuration)
        missing_configs = required_config - current_config

        # If there are no missing configs, return True
        return not bool(missing_configs)

    def install_dependencies(self):

        # Skip installation if in source_mode
        if self.reactor.source_mode:
            return gs.SUCCEEDED

        actdef_filename_list = [filename for filename in
                                os.listdir(self.reactor.actdef_folder) if
                                os.path.isfile(os.path.join(
                                    self.reactor.actdef_folder, filename))]
        dependencies = set()
        # Crawl through actdef files
        for filename in actdef_filename_list:
            with open(os.path.join(self.reactor.actdef_folder, filename)) as f:
                actdef = json.load(f)
            if 'required_pkgs' in actdef:
                for pkg in actdef['required_pkgs']:
                    dependencies.add(pkg)

        if dependencies:
            # Install (with default yes)
            resp = self.daemon_install(dependencies)
            # Goalstates don't have a negative value (they are uints) so this
            # will let us handle the wonky daemon process
            
            if resp:
                return -1
            else:
                return gs.ABORTED

        else:
            return gs.SUCCEEDED

    ## Runs sudo apt-get install in a daemon child process.
    #  This is achieved by forking the parent process twice to create an
    #  orphaned child process without a controlling terminal. As a result, we
    #  need to store the "grandchild's" PID to monitor for completeness.
    #  This code was largely adapted from Chad Schroeder's python daemon code
    #  (http://code.activestate.com/recipes/278731-creating-a-daemon-the-python-way/)
    def daemon_install(self, dependencies):
        try:
            # Create the first process
            pid = os.fork()
        except OSError as e:
            # Making a fork caused an error, so handle it here
            rospy.logwarn("Failed to create child for daemon setup: %s [%s]",
                          e.strerror, e.errno)
            return False

        # Code in the following block will only be executed by the first child
        # and its children
        if pid == 0:
            # Sets the child process as the session leader of the new session
            # and the process group leader of the new process group.
            os.setsid()
            try:
                # Fork a second child and exit the first child to orphan it.
                pid = os.fork()
            except OSError as e:
                # Making the second fork caused an error, so handle it here
                rospy.logwarn("Failed to create daemon grandchild: %s [%s]",
                              e.strerror, e.errno)
                return False

            # Code in the following block will only be executed by the second
            # child.
            if pid == 0:
                # Change the directory to the root directly to avoid unmounting
                # issues at shutdown time, in case the current working dir is
                # a mounted filesystem
                os.chdir("/")
                # Give the child complete control over permissions
                os.umask(0)
                # Install in the daemon
                subprocess.call(['sudo', 'apt-get', 'install', '-y'] +
                                list(dependencies))
                # Exit the daemon process (in case we didn't restart)
                rospy.logdebug("Exiting the second UPDATE child process...")
                os._exit(0)
            else:
                # Exit the first child. Does not call functions registered
                # with atexit/on_exit or signal handlers.
                rospy.logdebug("Exiting the first UPDATE child process...")
                os._exit(0)
        else:
            # Child's child's PID is the child's PID + 1
            self.daemon_pid = pid + 1
            # Original code had the parent exit, but we want it to stay running
            # so that we don't clip our update.
            return True

## @}
