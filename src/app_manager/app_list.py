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
#
# Revision $Id: topics.py 11753 2010-10-25 06:23:19Z kwc $

# author: kwc

"""
Implements applist part of app_manager, which handles listing of
currently installed applications.
"""

import os
import rospy
import sys
import yaml

from .app import load_AppDefinition_by_name
from .msg import App, ClientApp, KeyValue, Icon
from .exceptions import AppException, InvalidAppException, NotFoundException

def get_default_applist_directory():
    """
    Default directory where applist configuration is stored.
    """
    return "/etc/robot/apps"

def dict_to_KeyValue(d):
    l = []
    for k, v in d.iteritems():
        l.append(KeyValue(k, str(v)))
    return l

def read_Icon_file(filename):
    icon = Icon()
    if filename == None or filename == "":
        return icon
    basename, extension = os.path.splitext(filename)
    if extension.lower() == ".jpg" or extension.lower() == ".jpeg":
        icon.format = "jpeg"
    elif extension.lower() == ".png":
        icon.format = "png"
    else:
        icon.format = ""
        return icon
    icon.data = open(filename, "rb").read()
    return icon

def AppDefinition_to_App(app_definition):
    a = App(name=app_definition.name, display_name=app_definition.display_name, icon=read_Icon_file(app_definition.icon))
    a.client_apps = []
    for c in app_definition.clients:
        a.client_apps.append(ClientApp(c.client_type,
                                       dict_to_KeyValue(c.manager_data),
                                       dict_to_KeyValue(c.app_data)))
    return a

class InstalledFile(object):
    """
    Models data stored in a .installed file.  These files are used to
    track installation of apps.
    """

    def __init__(self, filename):
        self.filename = filename
        # list of App
        self.available_apps = []

        self._file_mtime = None
        self.update()

    def _load(self):
        available_apps = []
        with open(self.filename) as f:
            installed_data = yaml.load(f)
            for reqd in ['apps']:
                if not reqd in installed_data:
                    raise InvalidAppException("installed file [%s] is missing required key [%s]"%(self.filename, reqd))
            for app in installed_data['apps']:
                for areqd in ['app']:
                    if not areqd in app:
                        raise InvalidAppException("installed file [%s] app definition is missing required key [%s]"%(self.filename, areqd))
                try:
                    available_apps.append(load_AppDefinition_by_name(app['app']))
                except NotFoundException as e:
                    rospy.logerr(e)
                    continue
                except Exception as e:
                    raise e

        self.available_apps = available_apps

    def update(self):
        """
        Update app list
        """
        s = os.stat(self.filename)
        if s.st_mtime != self._file_mtime:
            self._load()
            self._file_mtime = s.st_mtime

    def get_available_apps(self, platform=None):
        if platform is not None:
            return filter(lambda app: app.platform == platform,
                          self.available_apps)
        else:
            return self.available_apps

    def __eq__(self, other):
        return self.filename == other.filename

    def __neq__(self, other):
        return not self.__eq__(other)


class AppList(object):
    def __init__(self, applist_directories, platform=None):
        self.applist_directories = applist_directories
        self.installed_files = {}
        self.invalid_installed_files = []
        self.app_list = []
        self.platform = platform

        self._applist_directory_mtime = None
        self.need_update = True

    def _find_installed_files(self):
        installed_files = []
        for d in self.applist_directories:
            for f in os.listdir(d):
                if f.endswith(".installed"):
                    full_path = os.path.abspath(os.path.join(d, f))
                    installed_files.append(full_path)
        return installed_files

    def _load(self, files):
        if files:
            installed_files = files
        else:
            installed_files = self.installed_files.keys()
        invalid_installed_files = []
        app_list = []
        for f in installed_files:
            try:
                if f in self.installed_files:
                    # update InstalledFile object
                    installed_file = self.installed_files[f]
                else:
                    # new installed file
                    installed_file = InstalledFile(f)
                    self.installed_files[f] = installed_file
                installed_file.update()
                app_list.extend(installed_file.get_available_apps(platform=self.platform))
                rospy.loginfo("%d apps found in %s" % (len(installed_file.available_apps), installed_file.filename))
            except AppException as e:
                rospy.logerr("ERROR: %s" % (str(e)))
                invalid_installed_files.append((f, e))
            except Exception as e:
                rospy.logerr("ERROR: %s" % (str(e)))
                invalid_installed_files.append((f, e))

        self.app_list = app_list
        self.invalid_installed_files = invalid_installed_files

    def get_app_list(self):
        if not self.app_list:
            self.update()
        return [AppDefinition_to_App(ad) for ad in self.app_list]

    def add_directory(self, directory):
        if not os.path.exists(directory):
            raise IOError("applist directory %s does not exist." % directory)
        if directory in self.applist_directory:
            raise RuntimeError("applist directory %s already exists" % directory)
        self.applist_directories.append(directory)
        self.need_update = True

    def remove_directory(self, directory):
        if directory not in self.applist_directory:
            raise RuntimeError("applist directory %s does not in list" % directory)
        self.applist_directory.remove(directory)
        self.need_update = True

    def update(self):
        """
        Update app list
        """
        files = None
        if self.need_update:
            files = self._find_installed_files()
            self.need_update = False
        self._load(files)
