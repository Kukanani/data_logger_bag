#!/usr/bin/env python

# Copyright (c) 2016, Socially Intelligent Machines Lab
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of data_logger_bag nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Vivian Chu, vchu@gatech.edu
# Modified: Adam Allevato, allevato@utexas.edu, November 2018

'''
Calls the command to write to bag files.
Allows for specification of where to store the bag files to as well as
the topics to record
'''

import rospy
import os
import time
import subprocess
import signal
from std_msgs.msg import Bool, String
from data_logger_bag.msg import LogControl
from data_logger_bag.srv import GetSettingsResponse, GetSettings

def terminate_ros_node(s):
    '''
    Useful function from ROS answers that kills all nodes with the string match
    at the beginning
    '''
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)

def ensure_dir(d):
    '''
    Simple function that creates all the directories in a path if they
    don't exist.

    d(string): the fully-qualified (no ~, etc.) directory path to
               check for existence. Note that this is a directory path,
               not a filename.
    '''
    if not os.path.exists(d):
        os.makedirs(d)

class DataLoggerBag:
    LOGGER_START_STOP_TOPIC = "~set_logging"
    GET_SETTINGS_SRV= "~get_settings"
    SET_SETTINGS_TOPIC= "~set_settings"
    OUTPUT_FILEPATH_TOPIC = "~last_bag_file_path"

    RECORD_TOPICS_PARAM = "~record_topics"
    LOG_DIRECTORY_PARAM = "~log_directory"

    def __init__(self):
        '''
        Initialize the ROS node, subscribers, and services, and load parameters
        '''

        ###
        # Initialize variables
        rospy.init_node("data_logger_bag", anonymous=True)


        self.is_logging = False
        self.log_prefix = ""
        self.log_directory = ""
        self.current_bag_filename = ""

        ###
        # Set up ROS pubs/subs/services/etc.

        # Load parameters
        # default topics to record - separated only by a space
        default_record_topics = ""
        default_log_directory = "~/data_logger_bag"
        self.record_topics = rospy.get_param(self.RECORD_TOPICS_PARAM, default_record_topics).split(" ")
        log_directory = rospy.get_param(self.LOG_DIRECTORY_PARAM, default_log_directory)
        self.set_log_directory(log_directory)

        # Setup publisher for bagfile
        self.bag_file_loc_pub = rospy.Publisher(self.OUTPUT_FILEPATH_TOPIC, String, queue_size=1)

        # Setup service for getting the current settings
        self.setting_srv = rospy.Service(self.GET_SETTINGS_SRV, GetSettings, self.cb_get_settings)

        # Setup a listener for the task and action to put in a custom folder
        # This message also contains which logging topics that the user can
        # specify to listen to
        rospy.Subscriber(self.SET_SETTINGS_TOPIC, LogControl, self.cb_set_settings)

        # Listen for the flag to know when to start and stop the recording
        rospy.Subscriber(self.LOGGER_START_STOP_TOPIC, Bool, self.cb_set_logging)

        ###
        # Ready to rumble
        rospy.loginfo("Initialized data logger bag node.")
        rospy.spin()


    def cb_get_settings(self, req):
        '''
        Setup service response to getting the current settings of the logger
        '''
        response = GetSettingsResponse()
        settings = LoggerSettings()
        settings.log_directory = self.log_directory
        settings.log_prefix = self.log_prefix

        response.current_settings = settings
        return response


    def cb_set_settings(self, msg):
        '''
        CB that changes the location of where we are saving data to based off of
        the current task and action
        '''

        # Only change the directory if we are not currently writing to a file

        if self.is_logging:
            rospy.logwarn("Currently recording, ignoring settings change.")
            return

        else:
            self.is_logging = True

            if msg.record_topics:
                self.record_topics = msg.record_topics

            if msg.log_prefix is not "":
                self.log_prefix = msg.log_prefix

            self.set_log_directory(msg.log_directory)

            # Append the task and skill onto the data_location
            rospy.loginfo("Location writing changed to: %s" % self.data_custom_location)

            # What topics we're recording
            rospy.loginfo("Topics that will be subscribed to: %s" % self.record_topics)


    def cb_set_logging(self, msg):
        '''
        Callback for when to trigger recording of bag files
        '''

        # Setup flag from message
        rospy.loginfo(rospy.get_name() + ": I heard %s" % msg.data)

        # Checks for change in flag first
        if self.is_logging != msg.data:
            # Set the flag after checking
            self.is_logging = msg.data

            # Then check what the message is telling us
            if msg.data:
                self.startRecord()
            else:
                self.stopRecord()
        elif msg.data:
            rospy.logwarn("recording requested, but already recording")
        else:
            rospy.logwarn("stop recording requested, but not recording")


    def set_log_directory(self, log_directory):
        """
        Do basic validation and set the logging directory.
        """

        if log_directory is not "":
            self.log_directory = \
                os.path.abspath(
                    os.path.expanduser(
                        os.path.expandvars(
                            log_directory
                        )
                    )
                )
            ensure_dir(log_directory)

    def startRecord(self):
        # construct the fully qualified filename from the log directory, prefix, and time.
        prefix = ""
        if self.log_prefix != "":
            prefix = self.log_prefix + "_"
        self.current_bag_filename = os.path.join(self.log_directory, prefix + time.strftime("%Y-%m-%dT%H%M%S") + ".bag")

        # Check if directory exists. We have to do this after constructing the filename, because
        # the parent directory we have to check depends on both the log directory and the prefix,
        # which may include additional folders.
        bag_directory = os.path.dirname(self.current_bag_filename)
        rospy.loginfo("ensuring that {} exists..".format(bag_directory))
        ensure_dir(bag_directory)

        # Setup the command for rosbag
        # We don't use the compression flag (-j) to avoid slow downs
        rosbag_cmd = " ".join(("rosbag record -O", self.current_bag_filename, " ".join(self.record_topics)))

        # Start the command through the system
        rospy.loginfo("Running the following record command: %s" % rosbag_cmd)
        self.rosbag_proc = subprocess.Popen([rosbag_cmd], shell=True)

    def stopRecord(self):
        # Send command to the process to end (same as ctrl+C)
        self.rosbag_proc.send_signal(subprocess.signal.SIGINT)
        rospy.loginfo("Stopping bag file recording")

        # Kill all extra rosbag "record" nodes
        terminate_ros_node("/record")

        # Publish out what bag file that we finished reccording to
        self.bag_file_loc_pub.publish(String(self.current_bag_filename))

if __name__ == '__main__':
    dlb = DataLoggerBag()
