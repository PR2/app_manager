#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

PKG = 'app_manager'

import os
import sys
import time
import unittest

import rospy
import rostest
import rosunit

from app_manager.srv import *
from std_msgs.msg import String

class StopAppTest(unittest.TestCase):

    def __init__(self, *args):
        super(StopAppTest, self).__init__(*args)
        rospy.init_node('stop_app_test')

    def cb(self, msg):
        rospy.logwarn("{} received".format(msg))
        self.msg = msg
        message = eval(msg.data)
        if ('start_plugin' in message
                and message['start_plugin']['fuga'] == 300
                and message['start_plugin']['hoge'] == 100):
            self.msg_started = True
        if ('stop_plugin' in message
                and 'exit_code' in message
                and 'stopped' in message
                and 'timeout' in message
                and message['stop_plugin']['fuga'] == 3000
                and message['stop_plugin']['hoge'] == 1000):
            self.msg_ctx_exit_code = message['exit_code']
            self.msg_ctx_stopped = message['stopped']
            self.msg_ctx_timeout = message['timeout']
            self.msg_stopped = True
        if ('param1' in message
                and 'param2' in message
                and message['param1'] == 'hello'
                and message['param2'] == 'world'):
            self.msg_plugin_started = True
        if ('param1' in message
                and 'param2' in message
                and message['param1'] == 'param1'
                and message['param2'] == 'param2'):
            self.msg_app_started = True
        self.msg_received = self.msg_received + 1

    def setUp(self):
        self.msg = None
        self.msg_received = 0
        self.msg_started = False
        self.msg_stopped = False
        self.msg_plugin_started = False
        self.msg_app_started = False
        self.msg_ctx_exit_code = None
        self.msg_ctx_stopped = None
        self.msg_ctx_timeout = None
        rospy.Subscriber('/test_plugin', String, self.cb)
        rospy.wait_for_service('/robot/list_apps')
        rospy.wait_for_service('/robot/start_app')
        rospy.wait_for_service('/robot/stop_app')
        self.list = rospy.ServiceProxy('/robot/list_apps', ListApps)
        self.start = rospy.ServiceProxy('/robot/start_app', StartApp)
        self.stop = rospy.ServiceProxy('/robot/stop_app', StopApp)

    def test_start_stop_app(self):
        # wait for plugins
        list_req = ListAppsRequest()
        list_res = ListAppsResponse()
        while not 'app_manager/test_plugin' in list(map(lambda x: x.name, list_res.available_apps)):
            list_res = self.list.call(list_req)
            # rospy.logwarn("received 'list_apps' {}".format(list_res))
            time.sleep(1)
        # start plugin
        start_req = StartAppRequest(name='app_manager/test_plugin')
        start_res = self.start.call(start_req)
        rospy.logwarn('start app {}'.format(start_res))
        self.assertEqual(start_res.error_code, 0)
        while (not rospy.is_shutdown()
                and not self.msg_started):
            rospy.logwarn('Wait for start message received..')
            rospy.sleep(1)

        # check app and plugin both started
        while (not rospy.is_shutdown()
                and not self.msg_app_started
                and not self.msg_plugin_started):
            rospy.logwarn('Wait for app/plugin message received..')
            rospy.sleep(1)

        # stop plugin
        stop_req = StopAppRequest(name='app_manager/test_plugin')
        stop_res = self.stop.call(stop_req)
        rospy.logwarn('stop app {}'.format(stop_res))
        self.assertEqual(stop_res.error_code, 0)

        while (not rospy.is_shutdown()
                and not self.msg_stopped):
            rospy.logwarn('Wait for stop message received..')
            rospy.sleep(1)

        self.assertEqual(self.msg_ctx_exit_code, None)
        self.assertEqual(self.msg_ctx_stopped, True)
        self.assertEqual(self.msg_ctx_timeout, None)

    def test_start_stop_app_timeout(self):
        # wait for plugins
        list_req = ListAppsRequest()
        list_res = ListAppsResponse()
        while not 'app_manager/test_plugin' in list(map(lambda x: x.name, list_res.available_apps)):
            list_res = self.list.call(list_req)
            # rospy.logwarn("received 'list_apps' {}".format(list_res))
            time.sleep(1)
        # start plugin
        start_req = StartAppRequest(name='app_manager/test_plugin_timeout')
        start_res = self.start.call(start_req)
        rospy.logwarn('start app {}'.format(start_res))
        self.assertEqual(start_res.error_code, 0)
        while (not rospy.is_shutdown()
                and not self.msg_started):
            rospy.logwarn('Wait for start message received..')
            rospy.sleep(1)

        # check app and plugin both started
        while (not rospy.is_shutdown()
                and not self.msg_app_started
                and not self.msg_plugin_started):
            rospy.logwarn('Wait for app/plugin message received..')
            rospy.sleep(1)

        while (not rospy.is_shutdown()
                and not self.msg_stopped):
            rospy.logwarn('Wait for stop message received..')
            rospy.sleep(1)

        self.assertEqual(self.msg_ctx_exit_code, None)
        self.assertEqual(self.msg_ctx_stopped, True)
        self.assertEqual(self.msg_ctx_timeout, True)


if __name__ == '__main__':
    try:
        rostest.run('stop_app_test', PKG, StopAppTest, sys.argv)
    except KeyboardInterrupt:
        pass
    print("{} exiting".format(PKG))
