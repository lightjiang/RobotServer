#!/usr/bin/env python

# Copyright 2016 Fetch Robotics Inc.
# Author: Michael Hwang

## @file reactor.py

# Standard Library
import json
import os
import requests
import signal
import socket
from datetime import datetime
from distutils.version import StrictVersion

# Third Party
import rospkg
import rospy
import tf
from actionlib_msgs.msg import GoalStatus as gs
from rospkg.common import PACKAGE_FILE
from rospkg.rospack import ManifestManager
from std_msgs.msg import String as StringMessage

# Fetch
from fetch_driver_msgs.msg import RobotState
from fetch_move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from client import FetchcoreClient
from fetchcore_client.dispatch import (Dispatch,
                                       DispatchBuildmap,
                                       DispatchLocalize,
                                       DispatchMonitoredNavigate,
                                       DispatchNavigate,
                                       DispatchUpdate,
                                       DispatchWaitfor)
from fetchcore_client.monitor import Monitor
from fetchcore_client.helper_methods import (convert_lane_annotations_to_lanes,
                                             get_timestamp,
                                             strpose_from_xyz_quat)
from fetchcore_client.rosout_logger import RosoutLogger
from fetchcore_client.settings import (CORE_SERVER_URL,
                                       TEST_CORE_SERVER_URL,
                                       TIMESTAMP_FORMAT)
from fetch_move_base_msgs.msg import Lanes
from fetchcore_msgs.srv import RegisterAction, DeregisterAction
from power_msgs.msg import BatteryState
from power_msgs.srv import BreakerCommand
from wifi_status.msg import WirelessList

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
FAILED_GOALSTATES = ['PREEMPTED', 'ABORTED', 'REJECTED', 'RECALLED']
TERMINAL_GOALSTATES = FAILED_GOALSTATES + ["SUCCEEDED"]
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

TASK_STATES_TERMINAL = [TASK_STATE_PREEMPT, TASK_STATE_CANCEL,
                        TASK_STATE_FAILURE, TASK_STATE_SUCCESS]
## @}

## A list of packages we're tracking versions for.
BASE_TRACKED_PACKAGES = ('fetchcore_client',)

## Hack: Built-in functions that use the generic dispatch that we don't want to
#  skip with client_wait_time = 0
GENERIC_BUILTINS = ["DOCK", "UNDOCK"]


## Logic and storage for converting Task Actions into ROS Actions.
class Reactor(object):

    ## Initialization method.
    #  Requires either of the following combinations of arguments:\n
    #  1. agentname, password\n
    #  2. client\n
    #  Providing a client will override the other arguments.
    ## @param agentname The agentname to retrieve tasks under
    ## @param password The corresponding password entry for agentname
    ## @param client An instance used in lieu of an agentname/password combo
    ## @param debug Whether to run the reactor in debug mode or not
    ## @param client_wait_time Sets the time to wait for action clients
    ##        to come up. If set to 0, this disables all non-builtin actions
    def __init__(self, agentname=None, password=None, client=None,
                 debug=False, client_wait_time=10):
        print("init----")
        self.agentname = agentname if agentname else socket.gethostname()
        self.password = password

        self.client_wait_time = client_wait_time

        # Exit flag. Set to True when exit() is called.
        self._exit_called = False

        # In test_mode, we make connect to a different port
        self.testing_mode = rospy.get_param("~test_mode", False)
        # In source, we won't try to install debians
        self.source_mode = 1#rospy.get_param("~source_mode", False)

        # Blue light ServiceProxy (HMI-only)
        self.light_serv = None

        ## FetchcoreClient instance that the Reactor will use to communicate
        if self.testing_mode:
            server_url = TEST_CORE_SERVER_URL
        else:
            server_url = CORE_SERVER_URL
        self.client = (client if client else
                       FetchcoreClient(
                           self.agentname, self.get_password(), url=server_url))
        # NOTE: There used to be an on_connect arg provided here, but it was
        # called too early. It will have to be called after class members
        # are instantiated to make it work.
        self.client.get_password = self.get_password
        self.client.get_request_timeout = self.get_request_timeout
        self.client.get_login_timeout = self.get_login_timeout
        self.client.connect(handle_auth=True)

        # Debug option
        self.debug = debug
        ## FetchRosoutLogger instance for sending ROS logs to server
        self.rosout_logger = RosoutLogger(self.client, self.debug)

        ## Whether we need to cancel actions on our next update as cleanup
        self.cancel_actions_on_next_update = False
        ## Whether we need to update tasks on our next update due to a previously failed attempt
        self.update_task_on_next_update = False

        # Agent document members
        self.actions = {}
        self.configuration = ["MOBILE"]
        self.action_feedback = {}

        # configuration-specific fields
        # TODO(future): Move into self.settings?
        self.collision_params = {}

        # Task-related members
        self._current_task = None
        self.current_task_id = None
        self.current_task_type = None
        self.update_loop_iter = 0

        # Location-related members
        self.localized = False
        self.map_frame = rospy.get_param("~global_frame_id", "map")

        # init settings
        self.settings = {}
        self.update_robot_settings()

        # retrieve agent doc
        self.update_from_agent_doc()

        ## Battery level is set to -1 until we get a reading from robot state
        self.battery_level = -1

        ## Wifi level is set to -1 until we get a reading from wifi_status
        self.wifi_strength = -1

        ## Keeps a counter so that we can process tasks when they change
        #  outside of the reactor.
        self._action_idx = 0

        ## Subprocess that hosts the map server and AMCL.
        self._map_server = None
        self.map_id = None

        ## The last time we polled for default_map
        self.last_default_map_poll = rospy.get_rostime()
        ## The last time the default_map was updated (should be datetime obj)
        self.last_default_map_datetime = None

        ## Minimum time (in seconds) we should wait before polling for the
        #  default map when untasked
        self.default_map_poll_period = rospy.Duration(30)

        ## Whether we are using a speed limit map or not
        self.use_speedlimit = True
        ## Whether we are using a keepout map or not
        self.use_keepout = True

        ## Dispatch dictionary. Holds instances of dynamically-loaded ROS
        # Action messages and clients for later use.
        self.dispatch = dict()

        # Source mode will not use global folders
        if self.source_mode:
            fetchcore_client_root = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                 "..", "..")
            self.map_folder = os.path.join(fetchcore_client_root, "maps")
            self.actdef_folder = os.path.join(fetchcore_client_root, "actions")
        else:
            # Folder containing action definitions
            self.actdef_folder = rospy.get_param("~actdef_folder",
                                                 "/etc/fetchcore/actions/")
            ## Folder to store maps in (as self-contained folders)
            self.map_folder = rospy.get_param("~map_folder",
                                              "/etc/fetchcore/maps/")

        # Monitors dict stored as action_name:monitors
        self.monitors = {}

        # List of versions to update
        self.versions = {}
        for package in BASE_TRACKED_PACKAGES:
            self.versions[package] = self.get_version_for_package(package)

        # Launch anything found in that folder
        self.load_action_definitions()

        ## Action registry services
        self.action_reg_service = None
        self.action_unreg_service = None

        ## Current task publisher
        self.task_pub = rospy.Publisher("~current_task_id",
                                        StringMessage,
                                        queue_size=1, latch=True)

        self.task_type_pub = rospy.Publisher("~current_task_type",
                                             StringMessage,
                                             queue_size=1, latch=True)

        ## Number of seconds to wait before timing out an action
        self.action_timeout = 15

        # BatteryState tracking
        self.is_charging = False
        self.battery_buffer = None
        self.battery_buffer_index = 0
        self.battery_buffer_size = 3
        self.battery_sub = rospy.Subscriber("/battery_state",
                                            BatteryState,
                                            self.process_battery_state)

        # Wifi Status tracking
        self.wifi_sub = rospy.Subscriber("/wifi_list",
                                         WirelessList,
                                         self.process_wifi_list)

        # Location-related members
        self.tf_listener = tf.TransformListener()
        self._local_thresh = [0.5, 0.5, 0.137]
        # TODO: Re-enable when AMCL localization threshold is fixed.
        # self.local_sub = rospy.Subscriber("/amcl_pose",
        #                                   PoseWithCovarianceStamped,
        #                                   self.process_localization_state)

        # RobotState tracking
        self.is_runstopped = False
        self.time_when_runstopped = None
        # we want to assume it's above the voltage threshold since undocking
        # while not docked is less harmful than moving while docked
        self.charger_voltage = 15
        print(000)
        self.rstate_sub = rospy.Subscriber("/robot_state",
                                           RobotState,
                                           self.process_robot_state)

        # Launch the action registry services
        print(001)
        self.initialize_action_registry()
        print(002)
        # Let the server know we're online
        self.agent_status = "IDLE"

        # publisher and subscriber for aux map spoofing
        self.keepout_map_pub = None
        self.speedlimit_map_pub = None

        # publisher for graph metadata from annotations
        self.lanes_pub = rospy.Publisher("map/lanes", Lanes, queue_size=1,
                                         latch=True)

        # Launch the map server + AMCL
        self.check_default_map(force_poll=True)

        # How long should we wait for weight from amcl? Default to 2 seconds
        self.amcl_timeout = rospy.get_param("~amcl_timeout", 2.0)

        # How old of a localization will we re-use?  Default to 5 minutes.
        self.localization_timeout = rospy.get_param("~localization_timeout",
                                                    60 * 5)

        # Find out if we are already localized on the map from earlier
        self.check_localization()

    ## @brief Get the password from a ROS param
    def get_password(self):
        if not self.password:
            # Wait for serial to publish before returning
            rate = rospy.Rate(5)
            serial = fail_text = "serial_failure"
            while serial == fail_text:
                serial = "383638323034510200280036"#rospy.get_param("/robot/serial", fail_text)
                if serial == fail_text: # Sleep on failure
                    rate.sleep()
            self.password = serial
        return self.password

    ## Get the action timeout from a ROS param
    def get_action_timeout(self):
        try:
            return float(rospy.get_param("~action_timeout",
                                         self.action_timeout))
        except ValueError:
            return self.action_timeout

    ## Get the login timeout from a ROS param
    def get_login_timeout(self):
        try:
            return float(rospy.get_param("~login_timeout", 5.0))
        except ValueError:
            # Somebody set this rosparam to a un-floatable value
            return 5.0

    ## Get the request timeout from a ROS param
    def get_request_timeout(self):
        try:
            return float(rospy.get_param("~request_timeout", 30.0))
        except ValueError:
            # Somebody set this rosparam to a un-floatable value
            return 30.0

    def __del__(self):
        # Should trigger map_server subprocess cleanup
        self.map_server = None

        # # Last-ditch actionserver subprocess cleanup
        # for _, action_server in self.action_servers.iteritems():
        #     action_server.send_signal(subprocess.signal.SIGINT)


    ## @name properties
    #  These are python properties, similar to accessors in other languages.
    #  They act the same as members, but can run code defined in getter and
    #  setter methods when they are accessed and set, respectively.
    ## @{

    @property
    def current_task(self):
        return self._current_task

    @current_task.setter
    def current_task(self, value):

        if value:

            if "_id" in value:
                del value["_id"]
            if "_rev" in value:
                del value["_rev"]

            # If assigning something but already have a current task
            if (self._current_task and
                    self._current_task['task_id'] != value['task_id']):
                # Reset iteration counter if task is new
                self._action_idx = 0
                self.cancel_actions_on_next_update = False
                # Set our state to working since we got a task
                self.agent_status = "WORKING"
            elif self._current_task:
                if value['task_status'] in TASK_STATES_TERMINAL:
                    # The task we were working on has been finished elsewhere
                    self.cancel_actions_on_next_update = True
                else:
                    # It is absolute that the robot's versions of an action's
                    # status and return_values are ground truth, and that we
                    # should only care about the updates if the given task has
                    # been retrieved in a terminal state.
                    for i, action in enumerate(self._current_task["actions"]):
                        value["actions"][i]["action_status"] = \
                            action["action_status"]
                        if action.get("return_values"):
                            value["actions"][i]["return_values"] = \
                                action["return_values"]

                    # Replace the task status in the newly assigned current
                    # task with our current version
                    value["task_status"] = self._current_task["task_status"]

            # Publish current task ID
            self.current_task_id = value['task_id']
            self.current_task_type = value['task_type']
            self.task_pub.publish(StringMessage(value['task_id']))
            self.task_type_pub.publish(StringMessage(value["task_type"]))
        else:
            # If assigning None to current task
            self._action_idx = 0
            self.cancel_actions_on_next_update = False
            self.current_task_id = None
            self.current_task_type = None
            # Publish empty string message to clear buffer
            self.task_pub.publish(StringMessage())
            self.task_type_pub.publish(StringMessage())
            # Set our state to IDLE since we don't have a task, but only if
            # we weren't already OFFLINE/RUNSTOP
            if self.agent_status not in ("OFFLINE", "RUNSTOP"):
                self.agent_status = "IDLE"
        self._current_task = value

    @property
    def map_server(self):
        return self._map_server

    @map_server.setter
    def map_server(self, value):
        if self._map_server:
            try:
                if isinstance(self._map_server, Monitor):
                    # Kill previous instance if it's a Monitor
                    try:
                        self._map_server.kill_process(wait=True)
                    except OSError:
                        logging.warn("Failed to kill non-existent map server?")
            except TypeError:
                # We hit a TypeError only during testing due to mock, so
                # ignore it. Proper mocking behavior will be necessary later.
                pass

        self._map_server = value

    ## @}


    ## @name agent_changes
    #  Methods related to updating parts of the agent document
    ## @{

    ## Pull down the latest agent document from the server
    ## @returns False if update was unsuccessful
    ## @returns True if update was successful
    def update_from_agent_doc(self):
        agent_doc = self.client.get_own_agent_doc()
        if not agent_doc:
            return False
        agent_settings = agent_doc.get("settings", {})

        doc_config = agent_doc.get("configuration")
        if doc_config and doc_config != self.configuration:
            self.configuration = agent_doc.get("configuration")

        # If we have a custom collision model in our settings, set it in params
        self.collision_params = agent_settings.get(
            "custom_collision_model", {})

        # If agent settings didn't have one, try to get one from config
        if not self.collision_params:
            # We don't have a custom collision model and our configs changed
            self.collision_params = self.update_collisions_by_config()
        elif self.collision_params:
            # We assigned the custom collision model, so delete it that we don't overwrite stuff
            del agent_settings["custom_collision_model"]

        # Update our settings with agent settings
        self.settings.update(agent_settings)

    ## Update the collisions for a given agent according to their config.
    def update_collisions_by_config(self):
        rospy.loginfo("Checking for valid collision models")
        collision_params = {}
        collision_model_settings = self.client.get_settings_for_component(
            "collision_models")

        if not collision_model_settings:
            return collision_params

        for config in self.configuration:
            # Find potential entries
            if config in collision_model_settings:
                # Grab the first valid model for now
                # TODO: Improve s/t we get "best" model
                collision_params = collision_model_settings[config]
                break

        return collision_params

    ## Update ros parameter settings from "robot" component settings from the server.
    def update_robot_settings(self):
        settings = self.client.get_settings_for_component("robot")

        if not settings:
            return

        self.settings.update(settings)

        # propagate known settings to rosparam
        if settings.get("request_timeout"):
            rospy.set_param("~request_timeout",
                            settings.get("request_timeout"))

        if settings.get("login_timeout"):
            rospy.set_param("~login_timeout",
                            settings.get("login_timeout"))

        if settings.get("max_path_wait_time"):
            rospy.set_param("~max_path_wait_time",
                            settings.get("max_path_wait_time"))

    ## Update the actions in this agent's document on the server
    def update_agent_doc_actions(self):
        # Grab current actions
        update_dict = {}
        update_dict["actions"] = self.actions
        update_dict["versions"] = self.versions

        # Save change to server
        self.client.update_own_agent_doc(**update_dict)

    ## @returns Version of **package**. See rosversion for original code:
    #           (https://github.com/ros-infrastructure/rospkg/blob/master/scripts/rosversion#L83-L101)
    @staticmethod
    def get_version_for_package(package):
        version = "<unversioned>"
        try:
            # hack to make it work with wet packages
            mm = ManifestManager(PACKAGE_FILE)
            path = mm.get_path(package)
            package_manifest = os.path.join(path, 'package.xml')
            if os.path.exists(package_manifest):
                from xml.etree.ElementTree import ElementTree
                try:
                    root = ElementTree(None, package_manifest)
                    version = root.findtext('version')
                except Exception:
                    pass
        except rospkg.ResourceNotFound:
            pass
        return version

    def on_connect(self):
        self.update_agent_doc_actions()
        self.update_from_agent_doc()

    ## @}


    ## @name callback_funcs
    #  Callback functions for various subscribers
    ## @{

    ## Process the battery state message and update attributes accordingly.
    def process_battery_state(self, msg):
        if self.battery_level == -1:
            # When the battery level isn't stored, initialize the buffer
            self.battery_buffer = [msg.charge_level] * self.battery_buffer_size
        else:
            # Update the buffer in sequence to save on complexity
            self.battery_buffer[self.battery_buffer_index] = msg.charge_level

        self.is_charging = msg.is_charging
        self.battery_level = (sum(self.battery_buffer) /
                              float(len(self.battery_buffer)))
        self.battery_buffer_index += 1
        self.battery_buffer_index %= 3

    ## Process the wireless list message and update the wifi level.
    def process_wifi_list(self, msg):
        if len(msg.stats) == 0:
            self.wifi_strength = -1
        else:
            self.wifi_strength = msg.stats[0].link_quality

    ## Process the robot state message and update attributes accordingly.
    def process_robot_state(self, msg):
        runstop_changed = (msg.runstopped != self.is_runstopped)

        if runstop_changed and not msg.runstopped:
            # If we just came out of runstop, we may not be localized anymore
            self.localized = False

            # Explicitly use runstopped timestamp here since the once in the
            # agent document keeps updating while runstopped
            self.check_localization(self.time_when_runstopped)

        if runstop_changed and msg.runstopped:
            # If we just entered runstop, make sure we report it to server
            self.time_when_runstopped = datetime.utcnow()

        self.is_runstopped = msg.runstopped
        self.charger_voltage = msg.charger.charger_voltage

        if runstop_changed and self.is_runstopped and self.current_task:
            self._update_current_task(task_status=TASK_STATE_CANCEL)
            self.cancel_actions_on_next_update = True

    # TODO: Re-enable when AMCL localization threshold is fixed.
    # ## Process the covariance given by AMCL over the /amcl_pose topic.
    # def process_localization_state(self, msg):
    #     # Get the covariance values for x-x, y-y, and yaw-yaw
    #     cov_array = [msg.pose.covariance[0],
    #                  msg.pose.covariance[7],
    #                  msg.pose.covariance[35]]
    #     # Check if each covariance value is below their respective threshold
    #     is_localized = all([cov <= self._local_thresh[idx] for idx, cov
    #                         in enumerate(cov_array)])
    #     if is_localized != self.localized:
    #         # localization state changed, update it
    #         rospy.logdebug("Localization state has changed to: %s.",
    #                        "LOCALIZED" if is_localized else "MISLOCALIZED")
    #         rospy.logdebug("Covariance array (xx, yy, tt): %s", cov_array)
    #         self.localized = is_localized

    ## @}


    ## @name action_registry_methods
    #  These methods work to create the Action Registry that allows dynamic
    #  loading and execution of custom actions, provided they are
    #  well-specified and dependencies are met.
    ## @{

    ## Starts the action registry services and waits for them to activate.
    ## @throws rospy.ServiceException if a service cannot be contacted
    def initialize_action_registry(self):

        self.action_reg_service = rospy.Service('~register',
                                                RegisterAction,
                                                self.add_action_def)

        self.action_unreg_service = rospy.Service('~deregister',
                                                  DeregisterAction,
                                                  self.remove_action_def)

    ## Add a new Action definition to the dispatch dictionary.
    def add_action_def(self, msg, update=True):
        try:
            # Assume that msg is a normal JSON string
            action_def = json.loads(msg)
        except TypeError:
            # It must actually be the service call, with a "definition"
            # field for a StringMessage
            action_def = json.loads(msg.definition.data)

        rospy.loginfo("Adding action %s", action_def['action_name'])

        ## Assign a version number if one isn't provided
        try:
            actdef_version = action_def['version']
        except KeyError:
            rospy.logerr("Action definition for %s provided without version.",
                         action_def['action_name'])
            return StringMessage("Action was not registered.")

        ## Whether the action has an entry in the actions dict or not
        action_is_registered = (action_def['action_name'] in self.actions and
                                action_def['action_name'] in self.dispatch)

        ## Whether the action is a new version or not.
        if not action_is_registered:
            action_is_new = True
        else:
            action_is_new = (StrictVersion(actdef_version) >
                             StrictVersion(
                                 self.actions[
                                     action_def['action_name']]['version']))

        ## Exit early to avoid processing if the action isn't new
        if not action_is_new:
            return StringMessage("Non-new action definition provided. "
                                 "Skipping registration.")

        # Get message type (format: "fetch_move_base_msgs/MoveBase")
        pkg_name, base_action = str(action_def['action_type']).split("/")
        base_action_goal = base_action + "Goal"
        base_action_result = base_action + "Result"

        # (Python-only) The package name needs a ".msg" added to the end
        pkg_name += ".msg"

        # The actual action message should have Action at the end to work
        base_action += "Action"

        action_topic = action_def['action_ns']

        client_is_valid = False

        monitor = self.monitors.get(action_def['action_name'])

        if action_def['action_name'] == 'NAVIGATE':

            # Create a NAVIGATE dispatcher client
            disp_client = DispatchNavigate(
                reactor=self,
                monitor=monitor,
                action_msg=MoveBaseAction,
                goal_msg=MoveBaseGoal,
                **action_def)

            client_is_valid = True

        elif action_def['action_name'] == 'MONITORED_NAVIGATE':

            # Create a NAVIGATE dispatcher client
            disp_client = DispatchMonitoredNavigate(
                reactor=self,
                monitor=monitor,
                action_msg=MoveBaseAction,
                goal_msg=MoveBaseGoal,
                **action_def)

            client_is_valid = True

        elif action_def['action_name'] == 'WAITFOR':

            # Create a WAITFOR dispatcher client
            disp_client = DispatchWaitfor(**action_def)
            client_is_valid = True

        elif action_def['action_name'] == 'BUILDMAP':

            # Create a BUILDMAP dispatcher client
            disp_client = DispatchBuildmap(
                reactor=self,
                **action_def)
            client_is_valid = True

        elif action_def['action_name'] == 'LOCALIZE':

            # Create a LOCALIZE dispatcher client
            disp_client = DispatchLocalize(
                reactor=self,
                **action_def)
            client_is_valid = True

        else:

            # The generic builtin stuff is not ideal but I don't want to
            # mess up the test suite right now.
            if (self.client_wait_time == 0 and action_def['action_name']
                    not in GENERIC_BUILTINS):
                rospy.logwarn("Non-builtins disabled")
                return StringMessage("Non-builtins disabled")

            # Dynamically load necessary modules for non-builtins
            try:
                msg_pkg = __import__(pkg_name, globals(), locals(),
                                     [base_action, base_action_goal,
                                      base_action_result], -1)
            except ImportError, e:
                rospy.logerr("Ran into an error importing a package %s "
                             "for %s: %s", pkg_name,
                             action_def['action_name'], e)
                return StringMessage(
                    "Could not add %s due to missing package for action." %
                    action_def['action_name'])
            action_msg = getattr(msg_pkg, base_action)
            goal_msg = getattr(msg_pkg, base_action_goal)
            result_msg = getattr(msg_pkg, base_action_result)
            # Instantiate the client
            disp_client = Dispatch(
                reactor=self,
                action_msg=action_msg,
                goal_msg=goal_msg,
                result_msg=result_msg,
                monitor=monitor,
                **action_def)

            if not disp_client.map_required:
                if (disp_client.monitor and
                        not disp_client.monitor.process_alive()):
                    # If map isn't required, it should be good to launch
                    # monitor (if one was provided)
                    disp_client.monitor.launch()

                client_is_valid = disp_client.wait_for_server(
                    self.client_wait_time)
            elif disp_client.map_required:
                rospy.loginfo("Skipping client connection check for %s due "
                              "to map unavailability.", disp_client.name)
                client_is_valid = True

        if client_is_valid:
            # Store dispatch client in dispatch dict
            self.dispatch[action_def['action_name']] = disp_client

            self.actions[action_def['action_name']] = {
                'version': action_def['version']}

            # Parse packages and add their version to our tracking
            for required_package in action_def.get("required_pkgs", []):
                if required_package.startswith('ros-'):
                    # Extract package name from ros-distro-package-name-here format
                    required_package = '_'.join(required_package.split("-")[2:])
                else:
                    continue
                version = self.get_version_for_package(required_package)
                self.versions[required_package] = version

            if update:
                self.update_agent_doc_actions()
            return StringMessage("Action registered.")
        else:
            rospy.logwarn(
                "%s Action client could not contact ActionServer specified at"
                " %s. %s functionality will be unavailable.",
                action_def['action_name'], action_topic,
                action_def['action_name'])
            return StringMessage("Unable to contact action server.")

    ## Remove an Action definition from the dispatch dict.
    def remove_action_def(self, msg, update=True):
        try:
            # Assume that msg has a "name" field for a StringMessage
            action_name = msg.name.data
        except AttributeError:
            # msg must instead be a regular string
            action_name = msg
        if action_name in self.dispatch:
            del self.dispatch[action_name]
            del self.actions[action_name]
            if update:
                self.update_agent_doc_actions()
            return StringMessage(action_name)

    ## Get the respective Dispatch for a given action type from the
    #  dispatch dict.
    ## @returns A Dispatch() instance, if it exists
    ## @returns None otherwise
    def _get_dispatch_for_action_type(self, action_type):
        return self.dispatch.get(action_type)

    ## @}


    ## @name control_functions
    #  These functions provide the high-level logic and control for the robot
    #  during operation.
    ## @{

    ## Starts the control loop. It will call the self.exit() function if it
    #  runs into any exceptions, but will otherwise run continously.
    def start(self):

        # Add exit on signals
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, self.exit)

        # TODO: Bump rate up to ~20hz and add iteration checks to prevent request overload
        r = rospy.Rate(20)
        try:
            while not rospy.is_shutdown():
                # Update location at ~10 Hz
                if self.update_loop_iter % 2 == 0 and not self._exit_called:
                    self.location_update()
                # Task updates at ~2 Hz
                if self.update_loop_iter % 10 == 0 and not self._exit_called:
                    self.update()
                self.update_loop_iter += 1
                self.update_loop_iter %= 20
                r.sleep()
        except KeyboardInterrupt, SystemExit:
            pass
        except Exception as exc:
            import traceback
            traceback.print_exc()
            raise exc
        finally:
            self.exit()

    ## Attempt to clean up leftover items in event of crash or termination.
    def exit(self, signum=None, frame=None):
        if signum:
            rospy.logwarn("Exiting with received signal: %s", signum)
        if not self._exit_called:
            self._exit_called = True
        else:
            # Return early to avoid multiple exits on multiple signals
            return

        # Turn off the light iff we have it
        self.set_light_state(False)

        # wind down server-related stuff
        self.agent_status = "OFFLINE"
        if self.current_task:
            try:
                # Check if current action we're working on exit is an UPDATE
                idx = min(self._action_idx, len(self.current_task["actions"]))
                current_action_is_update = (
                    self.current_task["actions"][idx]["action_name"] ==
                    "UPDATE")
            except (KeyError, IndexError):
                # Either no actions or the index is too large
                current_action_is_update = False

            if current_action_is_update:
                # We're in the middle of an UPDATE, so it's a special case
                disp = self._get_dispatch_for_action_type("UPDATE")
                if (disp.client.daemon_pid and
                        os.path.exists("/proc/%s" % disp.client.daemon_pid)):
                    # We called an exit with the daemon process running and
                    # the daemon_pid existing
                    self._end_current_task(TASK_STATE_SUCCESS)
                else:
                    # We called an exit without a daemon process running
                    self._end_current_task(TASK_STATE_FAILURE,
                                           overwrite_terminal=False)
            else:
                self._end_current_task(TASK_STATE_FAILURE,
                                       overwrite_terminal=False)
        # High-velocity data update
        self.location_update()

        # wind down map server
        self.map_server = None

        # wind down actions
        for action_type in self.dispatch.keys():
            client = self._get_dispatch_for_action_type(action_type)
            client.cancel_all_goals()
        if self.action_reg_service:
            self.action_reg_service.shutdown('Reactor exit.')
        if self.action_unreg_service:
            self.action_unreg_service.shutdown('Reactor exit.')

        # shut down subscribers
        self.rstate_sub.unregister()
        self.battery_sub.unregister()

        # wind down action servers
        for monitor in self.monitors.values():
            monitor.kill_process(wait=True)

        # Log out from the server
        self.client.disconnect()

    ## Iterator function for updating location during the control loop. It can
    #  be called by itself to process a single cycle.
    def location_update(self):

        # Make a location array
        loc = {'agent_status': self.agent_status,
               'battery_level': self.battery_level,
               'localized': self.localized,
               'current_task_id': self.current_task_id,
               'current_task_type': self.current_task_type,
               'charging_state': self.is_charging,
               'wifi_strength': self.wifi_strength}
        # Find location on the map
        try:
            xyz, quat = self.tf_listener.lookupTransform(self.map_frame,
                                                         'base_link',
                                                         rospy.Time(0))
            pose_str = strpose_from_xyz_quat(xyz, quat)
            loc['current_pose'] = pose_str

        except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
            # this means TF isn't set up, so just don't include the pose
            pass

        loc['feedback'] = self.action_feedback

        # Push the latest amcl weight into loc if applicable
        self.update_amcl_weight(loc)

        # RUNSTOP status overrides location update unless OFFLINE
        if self.agent_status != "OFFLINE" and self.is_runstopped:
            loc['agent_status'] = "RUNSTOP"

        # We shouldn't be WORKING without a task ID
        if self.agent_status == "WORKING" and not self.current_task_id:
            self.agent_status = "IDLE"
            loc['agent_status'] = self.agent_status

        # Send location array to server
        try:
            self.client.update_own_location(**loc)
        except requests.exceptions.HTTPError, e:
            if self.debug:
                raise e

    ## Iterator function called during the control loop. It can be called
    #  by itself to process a single cycle.
    def update(self):
        try:
            # Grab a task
            self.current_task = self._get_task_from_server()
            if self.current_task:
                # TODO: Add processing if the task status has changed
                # TODO: Add processing for PREEMPTED vs CANCELED task statuses
                # TODO: Get processing at ~20hz but server requests ~1hz
                if self.cancel_actions_on_next_update:
                    # Need to clean up actions for actions that we touched
                    # and mark other actions as CANCELED
                    for idx, action in enumerate(self.current_task['actions']):
                        if action['action_status'] not in TASK_STATES_TERMINAL:
                            action['action_status'] = (
                                self.current_task['task_status'])
                        if idx <= self._action_idx:
                            self._clean_up_action(action)
                    # Send back the task and wrap it up
                    self._end_current_task(self.current_task['task_status'])
                else:
                    if self.current_task['task_status'] == "QUEUED":
                        # When first starting the task
                        # Check for default_map and renew if necessary
                        self.check_default_map(force_poll=True)
                        # Update robot settings before execution
                        self.update_robot_settings()
                        # Overwrite with agent settings before execution
                        self.update_from_agent_doc()
                    if self.update_task_on_next_update:
                        # If we failed to update a task, try updating it again now.
                        self._update_current_task()
                    # Process the task for actions, starting from the last
                    # known index.
                    for action in self.current_task['actions'][self._action_idx:]:
                        if action['action_status'] in TASK_STATES_TERMINAL:
                            # Getting to this point means that this action needs
                            # to be dealt with.
                            self._clean_up_action(action)
                            self._action_idx += 1
                        elif action["action_status"] == TASK_STATE_ACTIVE:
                            self._check_action_status(action)
                            break
                        elif action["action_status"] == TASK_STATE_PENDING:
                            self._start_action(action)
                            break
                        else:
                            rospy.logwarn("Action status %s unrecognized. "
                                          "Skipping %s action.",
                                          action['action_status'],
                                          action['action_name'])
                            action['action_status'] = TASK_STATE_FAILURE
                            self._action_idx += 1
                            self._end_current_task(TASK_STATE_FAILURE)
                            break
            else:
                # We are untasked, see if map has changed
                self.check_default_map()

                # Check if we need to make an update task for ourselves
                if self.source_mode and self.client._is_connected:
                    dir = self.actdef_folder
                    action_files = [f for f in os.listdir(dir)
                                    if os.path.isfile(os.path.join(dir, f))
                                    and f.endswith(".json")]
                    if len(action_files) == 0:
                        action = {
                            "action_name": "UPDATE",
                            "preemptable": "NONE",
                            "action_status": "QUEUED"
                        }
                        task = {
                            "task_type": "SOURCE-UPDATE",
                            "task_status": "NEW",
                            "assigned_agent": self.agentname,
                            "actions": [action]
                        }
                        self.client.create_task(**task)

        except requests.exceptions.HTTPError as e:
            rospy.logwarn("Ran into unexpected HTTP error: %s", e)
            if str(e).startswith('401') or str(e).startswith('404'):
                # It was an unauthorized request on what should be passing,
                # so assume we are PENDING auth.
                # TODO: Potentially move this logic to client into individual methods by checking agentname
                self.client._is_connected = False

    ## @}

    ## Assumes that self.last_default_map_poll and self.default_map_poll_period
    #  are not None
    def check_default_map(self, force_poll=False):

        # Check if we need to poll for the default_map
        current_rostime = rospy.get_rostime()
        do_poll = (True if force_poll else
                   (current_rostime - self.last_default_map_poll >=
                    self.default_map_poll_period))

        if force_poll:
            rospy.loginfo("Checking for default map updates (FORCED)")

        if do_poll:
            self.last_default_map_poll = rospy.get_rostime()

            if not self.client._is_connected:
                rospy.loginfo("Client not connected, skipping checking map")
                return

            try:
                default_map_id, default_map_timestamp = \
                    self.client.get_default_map_id(include_timestamp=True)
            except ValueError as err:
                # No default map set ?
                if str(err).startswith("Default"):
                    rospy.logdebug("'default_map' is not set")
                    return
                else:
                    raise err

            if default_map_id != self.map_id or \
                    self.last_default_map_datetime == None or \
                    datetime.strptime(default_map_timestamp, TIMESTAMP_FORMAT) > self.last_default_map_datetime:
                # Renew if default map's ID has changed
                self.get_map(default_map_id, default_map_timestamp)

    ## @name task_processing
    #  These methods all interact with task documents in some way or another.
    ## @{

    ## Get latest information for the current task or a new one entirely.
    def _get_task_from_server(self):
        if self.current_task:
            task = self.client.get_task(self.current_task['task_id'])
            if not task:
                # The task we're currently working on either doesn't exist
                # in the DB or we lost connection, keep the current task so
                # we don't stop working on it.
                task = self.current_task
        else:
            task = self.client.get_queued_task()
            # If we get a NEW status via race condition, do nothing this cycle
            if task and task['task_status'] == "NEW":
                task = None
        return task

    ## Upload the current task's changes to the database.
    ## @param kwargs If provided, update the given fields in the task before
    #  upload.
    ## @returns The response of the task update request
    def _update_current_task(self, **kwargs):
        # Access the private member to not interfere with property
        self._current_task.update(kwargs)
        try:
            resp = self.client.update_task(**self._current_task)
            # If resp is None, then we want to update it on the next update loop, but if it isn't we don't.
            self.update_task_on_next_update = resp is None
            return resp
        except requests.exceptions.HTTPError as err:
            if str(err).startswith('409'):
                # Resolve database conflict
                task = self.client.get_task(self._current_task['task_id'])
                task.update(kwargs)
                self.client.update_task(**task)
                self._current_task = task
            else:
                raise err

    ## End the current task with a given status. If the task update is
    #  successful, it unassigns the current task. Otherwise, it will keep the
    #  current task.
    ## @param status The status to end the current task with
    ## @param overwrite_terminal Whether to overwrite the status, even if it's
    #  terminal.
    def _end_current_task(self, status, overwrite_terminal=True):
        if (not overwrite_terminal and
                self.current_task['task_status'] in TASK_STATES_TERMINAL):
            # If we're not overwriting terminal status and it is terminal
            status = self.current_task['task_status']

        if status == TASK_STATE_FAILURE:
            # When a task is marked as FAILED, fail unterminated actions
            for action in self.current_task['actions']:
                if action['action_status'] not in TASK_STATES_TERMINAL:
                    action['action_status'] = TASK_STATE_FAILURE

        rospy.logdebug("Ending current task with status: %s.", status)
        resp = self._update_current_task(task_status=status)
        if resp:
            # Remove current task from memory if it was successfully updated
            self.current_task = None

    ## Start an action as specified by **action**.
    ## @param action The action to start.
    ## @param goal A goal object, if it has been pre-populated
    def _start_action(self, action, goal=None):
        # TODO: Check for required packages before execution?
        client = self._get_dispatch_for_action_type(action['action_name'])
        if client:
            start_success = client.start(goal, **action)
            if start_success:
                rospy.logdebug("Success fully started %s action.",
                               action['action_name'])
                action['action_status'] = TASK_STATE_ACTIVE
                action['start_timestamp'] = datetime.utcnow().strftime(
                    TIMESTAMP_FORMAT)
                self.agent_status = "WORKING"
                # TODO: This might actually be bad. Look into this later.
                self._current_task['task_status'] = TASK_STATE_ACTIVE
                self._update_current_task()
            else:
                logmsg = ("Failed to start action %s: action goal was not "
                          "sent successfully." % action["action_name"])
                rospy.logwarn(logmsg)
                action['action_status'] = TASK_STATE_FAILURE
                # Add diagnostic message for this specific error
                action['message'] = logmsg
                nowstamp = datetime.utcnow().strftime(TIMESTAMP_FORMAT)
                action['start_timestamp'] = action['end_timestamp'] = nowstamp
                self._end_current_task(TASK_STATE_FAILURE)
        else:
            logmsg = ("Failed to start action %s: action type was undefined."
                      % action["action_name"])
            rospy.logwarn(logmsg)
            action['action_status'] = TASK_STATE_FAILURE
            # Add diagnostic for this specific error
            action['message'] = logmsg
            nowstamp = datetime.utcnow().strftime(TIMESTAMP_FORMAT)
            action['start_timestamp'] = action['end_timestamp'] = nowstamp
            self._end_current_task(TASK_STATE_FAILURE)

    ## Check and update an action's status
    ## @param The task-action to check
    def _check_action_status(self, action):
        client = self._get_dispatch_for_action_type(action['action_name'])
        status = client.get_status()
        if client.monitor and not client.monitor.process_alive():
            # Our process monitor for this action is reporting our process as dead
            logmsg = "Process for %s action has died."
            rospy.logwarn(logmsg + " Failing action for %s", action["action_name"], self.current_task_id)
            action["message"] = logmsg % action["action_name"]
            status = TASK_STATE_FAILURE

        # Check if we've timed out on our action
        if status not in TASK_STATES_TERMINAL and client.last_update_received and client.can_timeout:
            # Save it here and use below because of a race condition; if the condition already hits, just assume we
            # got a last update now
            last_update_received = client.last_update_received or rospy.get_rostime()
            try:
                # Get a try-except here just in case it goes wonky again.
                past_timeout = rospy.get_rostime() > (last_update_received + rospy.Duration(self.get_action_timeout()))
            except TypeError:
                past_timeout = False

            if past_timeout:
                # We've gone over the timeout threshold, so set the status to # FAILED for action
                logmsg = ("%s action for %s has timed out. (Threshold %.2fs)."
                          % (action["action_name"], self.current_task_id, self.get_action_timeout()))
                rospy.logwarn(logmsg)
                action_msg = ("%s action has timed out. (Threshold %.2fs)."
                              % (action["action_name"], self.get_action_timeout()))
                action["message"] = action_msg
                status = TASK_STATE_FAILURE

        # Only send update if the status has changed
        if status != action['action_status']:
            action['action_status'] = status
            if status in TASK_STATES_TERMINAL:
                # We've finished the action
                action['end_timestamp'] = get_timestamp()
                if client.return_values:
                    # We have return values, so add them to the action section
                    action["return_values"] = client.return_values
                self._action_idx += 1

                # Cancel actions for this ended action to prevent accidental
                # continuation by ROS
                client.cancel_all_goals()

                if status == TASK_STATE_FAILURE:
                    # If we've FAILED the task
                    self._end_current_task(status)
                    # Return early since we don't want to do any more task
                    # processing
                    return
            if self._action_idx >= len(self._current_task['actions']):
                # This action is the last one, so end it with the same status
                # as the latest action.
                self._end_current_task(action['action_status'])
            else:
                self._update_current_task()

    ## Tells client relevant to action to stop executing.
    def _clean_up_action(self, action):
        try:
            client = self._get_dispatch_for_action_type(action['action_name'])
            client.cancel_all_goals()
        except AttributeError, e:
            rospy.logwarn(e)
            rospy.logwarn("Attempted to cancel all goals for a nonexistent "
                          "or improperly implemented Dispatch client. "
                          "(%s action)", action['action_name'])

    ## @}

    ## @name startup
    #  Clientside methods that run once when this class is initialized.
    ## @{

    ## Load action definitions (previously retrieved) in self.actdef_folder
    def load_action_definitions(self):

        if not os.path.exists(self.actdef_folder):
            os.makedirs(self.actdef_folder)

        # For storing action definitions
        actdef_dict = dict()

        # For storing launch files and their parameters
        launch_dict = dict()

        # Get names of all files located in self.actdef_folder
        actdef_filename_list = [filename for filename in
                                os.listdir(self.actdef_folder) if
                                os.path.isfile(os.path.join(
                                    self.actdef_folder, filename))]
        # Sort the paths
        actdef_filename_list = sorted(actdef_filename_list)

        # Get blacklisted actions
        action_blacklist = self.client.get_setting_for_component(
            "update", "action_blacklist")

        if not action_blacklist:
            action_blacklist = []

        # Load and process each action definition
        for filename in actdef_filename_list:
            filepath = os.path.join(self.actdef_folder, filename)
            try:
                with open(filepath) as f:
                    actdef = json.load(f)
            except ValueError:
                rospy.logwarn("Could not decode valid JSON when attempting to "
                              "load action file %s.", filepath)
                continue
            except IOError as e:
                rospy.logwarn("Could not open action definition file %s: %s",
                              filepath, e)
                continue

            if not actdef:
                # We got a null file
                rospy.logwarn("Received a null action definition when "
                              "attempting to load %s. Now cleaning up empty "
                              "file.", filepath)
                if os.path.isfile(filepath):
                    os.unlink(filepath)
                continue

            # Don't load actions that are in the blacklist, in case we
            # downloaded them on accident
            try:
                if actdef["action_name"] in action_blacklist:
                    continue
            except KeyError:
                rospy.logwarn("Could not load action definition due to "
                              "missing 'action_name' key.")
                continue

            # Store them in actdef_dict under their action name
            actdef_dict[actdef['action_name']] = actdef
            # Store them in launch_dict under package name, launch file,
            # and launch parameters
            try:
                launch_pkg, launch_file = actdef['launch_file'].split("/")
            except KeyError:
                rospy.logwarn("Could not load action definition %s due to "
                              "missing 'launch file' field.", filename)
                # Remove their entry from the actdef since it won't work
                del actdef_dict[actdef['action_name']]
                continue

            if launch_pkg == "builtin":
                # Skip launch file processing if it's a "builtin" launch
                continue

            # Collate parameters specified for launch file
            params = dict()
            if 'launch_file_parameters' in actdef:
                params = actdef['launch_file_parameters']

            if launch_pkg not in launch_dict:
                # This launch package is brand new
                rospy.logdebug("Adding %s %s to launch file list.",
                               launch_pkg, launch_file)
                launch_dict[launch_pkg] = {
                    launch_file: {'params': params,
                                  'actions': [actdef['action_name']]}}
            else:
                if launch_file not in launch_dict[launch_pkg]:
                    # This launch file is brand new, but not the package
                    rospy.logdebug("Adding %s %s to launch file list.",
                                   launch_pkg, launch_file)
                    launch_dict[launch_pkg][launch_file] = {
                        'params': params, 'actions': [actdef['action_name']]}
                else:
                    # This launch file is not brand new, so just update params
                    # and add action
                    for k, v in params.iteritems():
                        # Updating field by field since dict.update() seems to
                        # overwrite fields not included
                        rospy.logdebug("Setting launch argument %s:=%s for"
                                       " %s %s", k, v, launch_pkg, launch_file)
                        launch_dict[launch_pkg][launch_file]['params'][k] = v
                    launch_dict[launch_pkg][launch_file]['actions'].append(
                        actdef['action_name'])

        # Launch the files in launch_dict
        for launch_pkg, launch_files in launch_dict.iteritems():
            for launch_file, properties in \
                launch_files.iteritems():
                # Gather parameters
                launch_file_params = []
                for arg, val in properties['params'].iteritems():
                    launch_file_params.append("%s:=%s" % (arg, val))

                # Construct monitor
                monitor = self.create_monitor(launch_pkg=launch_pkg,
                                              launch_file=launch_file,
                                              launch_params=launch_file_params)
                # Assign same monitor for each action that requires the
                # launch file
                for actname in properties['actions']:
                    self.monitors[actname] = monitor

        for _, actdef in actdef_dict.iteritems():
            # Add action the normal way (needs JSON string input)
            self.add_action_def(json.dumps(actdef), update=False)

        # Add special builtin UPDATE action
        upd_disp_client = DispatchUpdate(
            action_name="UPDATE",
            map_required=False,
            can_timeout=False,
            reactor=self,
            optional_fields={"sources": "sources",
                             "override_sources": "override_sources",
                             "aptkeys": "aptkeys"})
        self.dispatch['UPDATE'] = upd_disp_client
        self.actions['UPDATE'] = {'version': '1.0.0'}

        self.update_agent_doc_actions()

    def create_monitor(self, launch_pkg=None, launch_file=None,
                       launch_params=None):
        return Monitor(self, launch_pkg=launch_pkg, launch_file=launch_file,
                       launch_params=launch_params)

    def get_map(self, map_id, timestamp):

        try:
            mapinfo = self.client.save_map_to_folder(map_id, self.map_folder)
            self.last_default_map_datetime = datetime.strptime(
                timestamp, TIMESTAMP_FORMAT)
        except requests.exceptions.HTTPError as err:
            if str(err).startswith('404'):
                rospy.logwarn("No map entry found when downloading %s",
                              map_id)
        except IOError as err:
            # Stopgap error message, see #659
            rospy.logerr("There was an error trying to save the map: %s.",
                         err)
            rospy.logerr("Make sure you have used fetchcore_config to setup or"
                         " run the fetchcore client in source_mode.")
            rospy.logerr("Exiting program.")
            # we don't use sys elsewhere
            import sys
            sys.exit()

        # TODO: maps will have a history/lineage, only delocalize if
        #       the new map is not derived from our current map
        if self.map_id != map_id:
            self.localized = False

        if mapinfo:
            self.map_id = str(map_id)
            self.launch_map_server()
            # Publish the metadata from annotations, if they exist
            if 'annotations' in mapinfo:
                annotations = mapinfo['annotations']
                if 'nodes' in annotations and 'edges' in annotations:
                    lanes = convert_lane_annotations_to_lanes(annotations)
                else:
                    # Publish a blank graph to clear the buffer
                    lanes = Lanes()
            else:
                # Publish a blank graph to clear the buffer
                lanes = Lanes()
            self.lanes_pub.publish(lanes)

    def launch_map_server(self):

        # Cleanup is handled automatically since this is a property
        self.map_server = None

        # Remove / from end of map_folder
        if self.map_folder.endswith("/"):
            map_folder = self.map_folder[:-1]
        else:
            map_folder = self.map_folder

        # Check if keepout and speedlimit are inside the folder before launch
        named_map_folder = os.path.join(self.map_folder, self.map_id)
        named_map_file_list = os.listdir(named_map_folder)
        self.use_keepout = 'keepout.yaml' in named_map_file_list
        self.use_speedlimit = 'speedlimit.yaml' in named_map_file_list

        # If our new map has a real keepout but we have an active spoofer,
        # shut it down so only one thing publishes a keepout.
        if self.use_keepout and self.keepout_map_pub:
            self.keepout_map_pub.unregister()
            self.keepout_map_pub = None
        # If our new map has a real speedlimit but we have an active spoofer,
        # shut it down so only one thing publishes a speedlimit.
        if self.use_speedlimit and self.speedlimit_map_pub:
            self.speedlimit_map_pub.unregister()
            self.speedlimit_map_pub = None

        # Map parameters from settings
        use_amcl = self.settings.get("use_amcl", False)

        self.map_server = self.create_monitor()
        self.map_server.launch("fetchcore_client",
                               "map_server.launch.xml",
                               ['maps_dir:=%s' % map_folder,
                                'map_name:=%s' % self.map_id,
                                'use_amcl:=%s' % str(use_amcl).lower(),
                                'use_speedlimit:=%s' % str(self.use_speedlimit).lower(),
                                'use_keepout:=%s' % str(self.use_keepout).lower()])

        # Launch processes that require the map, have a monitor, but might not
        # have been started.
        for dispatch in self.dispatch.values():
            if (dispatch.map_required
                    and dispatch.monitor
                    and not dispatch.monitor.process_alive()):
                dispatch.monitor.launch()

        # A new map means a new action server for localization checking
        if 'LOCALIZE' in self.dispatch:
            self.dispatch['LOCALIZE'].reset_check_pose_client()

    # If the robot is not localized, try the last localized position in the
    # agent doc and see if it's still valid and sane. If so, set localized.
    # prev_location_timestamp will override the one from the agent doc.
    def check_localization(self, prev_location_timestamp=None):
        if self.localized:
            rospy.loginfo("Robot is already localized. "
                          "No need to check localization.")
            return

        if 'LOCALIZE' not in self.dispatch:
            rospy.logwarn("No localize dispatch found, "
                          "cannot update localization.")
            return

        agent_doc = self.client.get_own_agent_doc()

        if prev_location_timestamp:
            try:
                agent_doc['last_updated_timestamp'] = \
                    prev_location_timestamp.strftime(TIMESTAMP_FORMAT)
            except TypeError:
                rospy.logwarn("Failed to get agent doc. "
                              "Cannot check localization.")
                return

            self.localized = \
                self.dispatch['LOCALIZE'].check_old_localization(
                    agent_doc, self.localization_timeout,
                    self.amcl_timeout)
        else:
            # Reactor just started. Give amcl some time to start up
            self.localized = \
                self.dispatch['LOCALIZE'].check_old_localization(
                    agent_doc, self.localization_timeout, 20.0,
                    on_start=True)

    # Check the location array for a pose and, if applicable, push on the
    # associated amcl weight
    def update_amcl_weight(self, loc):
        # Position and amcl weight should not update when runstopped
        if self.is_runstopped:
            return

        pose_str = loc.get('current_pose')
        if not pose_str:
            return

        if 'LOCALIZE' not in self.dispatch:
            return

        try:
            x, y, theta = [float(f) for f in pose_str.split(",")]
        except (ValueError, AttributeError) as e:
            return

        # TODO(enhancement): get weight in /amcl_pose callback instead
        amcl_weight = self.dispatch['LOCALIZE'].get_amcl_weight(x, y, theta)

        # Don't clobber latest weight with None on shutdown
        if amcl_weight:
            loc['amcl_weight'] = amcl_weight

    ## @}

    ## Sets the light state to **state** using self.light_serv. This should
    #  not be used on robots without the LED lights set up on aux_breaker_1,
    #  since toggling those breakers could have big consequences.
    ## @param state The state of the light: True for on, False for off.
    def set_light_state(self, state):
        # TODO: Consider moving this out of reactor in case certain dispatches that require it do not reference this class. Perhaps move it into a var or something?

        # Start up the service proxy for the first time, but only if it's a
        # signal to turn ON
        if state and not self.light_serv:
            rospy.loginfo("Starting up light service for the first time...")
            self.light_serv = rospy.ServiceProxy("/aux_breaker_1",
                                                 BreakerCommand)
            self.light_serv.wait_for_service()

        if self.light_serv:
            try:
                self.light_serv(bool(state))
            except rospy.ServiceException:
                rospy.logwarn("Ran into service exception when trying to set "
                              "light service to %s", state)

if __name__ == "__main__":

    # TODO: Potentially move loop control to here for easy SIGTERM?
    rospy.init_node("reactor")

    reactor = Reactor()
    print("123")
    reactor.start()