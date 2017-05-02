#!/usr/bin/env python

# Copyright 2016 Fetch Robotics Inc.
# Author: Michael Hwang

# Standard Library
import os
import subprocess

# Third Party
import rospy

# Fetch

## @file monitor.py Code for the Monitor class file

## Wrapper class for handling subprocesses.
class Monitor(object):

    ## @param reactor The Reactor instance this is spawned from
    ## @param proc The process we're monitoring
    def __init__(self, reactor, proc=None, launch_pkg=None, launch_file=None,
                 launch_params=None):
        self.reactor = reactor
        self.proc = proc

        # Launch-related members, set by self.launch()
        self.launch_pkg = launch_pkg
        self.launch_file = launch_file
        self.launch_params = launch_params
        self.shutdown_params = ["fetchcore_kill_postscript:=true"]

        # The max amount of time to poll when starting up a subprocess
        self.startup_poll_timeout = 2.0
        # The rate (in hz) to poll during the startup
        self.startup_poll_rate = 2

    ## Determine whether contained subprocess is still running.
    ## @returns True if process is still running
    ## @returns False if process has ended
    def process_alive(self):
        if self.proc:
            return self.proc.poll() is None
        else:
            return False

    ## For execution when this class is garbage-collected
    def __del__(self):
        if self.proc:
            self.kill_process()

    ## @brief Actually start the launchfile
    ## @param launch_pkg ROS package containing this launch file
    ## @param launch_file Launch file name to start
    ## @param launch_file_params Additional parameters to pass to roslaunch
    def launch(self, launch_pkg=None, launch_file=None,
               launch_file_params=None, wait_on_kill=False,
               wait_on_startup=True):

        # Make sure we don't launch more than one instance
        if self.proc:
            self.kill_process(wait_on_kill)

        # Get defaults unless they've been given as params
        launch_pkg = launch_pkg if launch_pkg else self.launch_pkg
        launch_file = launch_file if launch_file else self.launch_file
        launch_file_params = (launch_file_params if launch_file_params
                              else self.launch_params)

        if not launch_pkg:
            rospy.logerr("Package not provided when launch was called.")
            return

        if not launch_file:
            rospy.logerr("File not provided when launch was called.")
            return

        cmd = ['roslaunch', launch_pkg, launch_file]
        if launch_file_params:
            cmd.extend(launch_file_params)
        cmd.extend(self.shutdown_params)

        # Overwrite related members
        self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        self.launch_pkg = launch_pkg
        self.launch_file = launch_file
        self.launch_params = launch_file_params

        # Startup poll to prevent race conditions
        if wait_on_startup:
            r = rospy.Rate(self.startup_poll_rate)
            timeout = rospy.get_rostime() + rospy.Duration(
                self.startup_poll_timeout)
            while not self.process_alive() or rospy.get_rostime() < timeout:
                r.sleep()

    ## Kill the contained process
    ## @param wait Whether we should wait for the process to end
    def kill_process(self, wait=False):
        if self.proc:
            try:
                self.proc.send_signal(subprocess.signal.SIGINT)
                if wait:
                    self.proc.wait()
            except OSError as e:
                # Hitting an OSError means we can't handle it regardless,
                # so just remove it from assignment so we don't hit it again.
                self.proc = None
                if str(e).startswith("[Errno 3]"):
                    # Means that trying to kill a nonexistent process
                    pass
                else:
                    raise e
