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

class StopAppTest(unittest.TestCase):

    def __init__(self, *args):
        super(StopAppTest, self).__init__(*args)
        rospy.init_node('stop_app_test')

    def setUp(self):
        rospy.wait_for_service('/robot/list_apps')
        rospy.wait_for_service('/robot/stop_app')
        self.list = rospy.ServiceProxy('/robot/list_apps', ListApps)
        self.stop = rospy.ServiceProxy('/robot/stop_app', StopApp)

    def test_stop_app(self):
        list_req = ListAppsRequest()
        list_res = ListAppsResponse()
        while not 'app_manager/appA' in list(map(lambda x: x.name, list_res.running_apps)):
            list_res = self.list.call(list_req)
            # rospy.logwarn("received 'list_apps' {}".format(list_res))
            time.sleep(1)

        stop_req = StopAppRequest(name='app_manager/appA')
        stop_res = self.stop.call(stop_req)
        rospy.logwarn("received 'stop_app'  {}".format(stop_res))
        self.assertEqual(stop_res.stopped, True)

if __name__ == '__main__':
    try:
        rostest.run('stop_app_test', PKG, StopAppTest, sys.argv)
    except KeyboardInterrupt:
        pass
    print("{} exiting".format(PKG))
