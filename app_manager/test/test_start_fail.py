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

from app_manager.msg import *
from app_manager.srv import *

from std_msgs.msg import String

class StartFailTest(unittest.TestCase):

    def __init__(self, *args):
        super(StartFailTest, self).__init__(*args)
        rospy.init_node('start_fail_test')

    def cb(self, msg):
        rospy.logwarn("{} received".format(msg))
        self.msg = msg
        self.msg_received = self.msg_received + 1

    def setUp(self):
        rospy.wait_for_service('/robot/list_apps')
        rospy.wait_for_service('/robot/stop_app')
        rospy.wait_for_service('/robot/start_app')
        self.list = rospy.ServiceProxy('/robot/list_apps', ListApps)
        self.stop = rospy.ServiceProxy('/robot/stop_app', StopApp)
        self.start = rospy.ServiceProxy('/robot/start_app', StartApp)
        self.msg = None
        self.msg_received = 0
        rospy.Subscriber('/chatter', String, self.cb)

    def test_start_fail_app(self):
        # wait for application
        rospy.logwarn("Wait for application")
        list_req = ListAppsRequest()
        list_res = ListAppsResponse()
        while not 'app_manager/appA' in list(map(lambda x: x.name, list_res.available_apps)):
            list_res = self.list.call(list_req)
            rospy.logwarn("received 'list_apps' {}".format(list_res))
            time.sleep(1)

        # intentionally failed to start
        rospy.logwarn("Start application with wrong arguments")
        start_req = StartAppRequest(name='app_manager/appA', args=[KeyValue('launch_prefix', 'no_command')])
        start_res = self.start.call(start_req)
        rospy.logwarn("start 'started_app'  {}".format(start_res))
        self.assertEqual(start_res.started, False)

        # confirm if application is failed to start
        rospy.logwarn("Check running application")
        list_req = ListAppsRequest()
        list_res = ListAppsResponse(running_apps=[App(name='app_manager/appA')])
        while 'app_manager/appA' in list(map(lambda x: x.name, list_res.running_apps)):
            list_res = self.list.call(list_req)
            rospy.logwarn("received 'list_apps' {}".format(list_res))
            time.sleep(1)
            break

        # start app and check if actually started
        rospy.logwarn("Start application")
        start_req = StartAppRequest(name='app_manager/appA', args=[KeyValue('launch_prefix', 'python{}'.format(os.environ['ROS_PYTHON_VERSION']))])
        start_res = self.start.call(start_req)
        rospy.logwarn("received 'started_app'  {}".format(start_res))
        self.assertEqual(start_res.started, True)

        # check if msg received
        while (not rospy.is_shutdown()) and self.msg == None:
            rospy.logwarn('Wait for /chatter message received..')
            rospy.sleep(1)

        # stop plugin
        stop_req = StopAppRequest(name='app_manager/appA')
        stop_res = self.stop.call(stop_req)
        rospy.logwarn('stop app {}'.format(stop_res))
        self.assertEqual(stop_res.stopped, True)

if __name__ == '__main__':
    try:
        rostest.run('start_fail_test', PKG, StartFailTest, sys.argv)
    except KeyboardInterrupt:
        pass
    print("{} exiting".format(PKG))
