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
# Revision $Id: app_manager.py 14948 2011-09-07 19:25:54Z pratkanis $

# author: leibs
import sys
import os

if sys.version_info[0] == 3:
    import _thread as thread  # python3 renamed from thread to _thread
else:
    import thread

import time
import yaml

import rosgraph.names
import rospy
import roslib

import roslaunch.config
import roslaunch.core
import roslaunch.parent
import roslaunch.pmon
import roslaunch.xmlloader

import roslaunch.loader
from std_srvs.srv import Empty, EmptyResponse

from .app import AppDefinition, find_resource, load_AppDefinition_by_name
from .exceptions import LaunchException, AppException, InvalidAppException, NotFoundException
from .master_sync import MasterSync
from .msg import App, AppList, StatusCodes, AppStatus, AppInstallationState, ExchangeApp
from .srv import StartApp, StopApp, ListApps, ListAppsResponse, StartAppResponse, StopAppResponse, InstallApp, UninstallApp, GetInstallationState, UninstallAppResponse, InstallAppResponse, GetInstallationStateResponse, GetAppDetails, GetAppDetailsResponse

# for profiling
# import cProfile, pstats
# from io import BytesIO as StringIO

def _load_config_default(
        roslaunch_files, port, roslaunch_strs=None, loader=None, verbose=False,
        assign_machines=True, ignore_unset_args=False
):
    config = roslaunch.config.ROSLaunchConfig()
    if port:
        config.master.uri = rosgraph.network.create_local_xmlrpc_uri(port)

    loader = loader or roslaunch.xmlloader.XmlLoader()
    loader.ignore_unset_args = ignore_unset_args

    # load the roscore file first. we currently have
    # last-declaration wins rules.  roscore is just a
    # roslaunch file with special load semantics
    roslaunch.config.load_roscore(loader, config, verbose=verbose)

    # load the roslaunch_files into the config
    for f in roslaunch_files:
        if isinstance(f, tuple):
            f, args = f
        else:
            args = None
        try:
            rospy.loginfo('loading config file %s' % f)
            loader.load(f, config, argv=args, verbose=verbose)
        except roslaunch.xmlloader.XmlParseException as e:
            raise roslaunch.core.RLException(e)
        except roslaunch.loader.LoadException as e:
            raise roslaunch.core.RLException(e)
    # we need this for the hardware test systems, which builds up
    # roslaunch launch files in memory
    if roslaunch_strs:
        for launch_str in roslaunch_strs:
            try:
                rospy.loginfo('loading config file from string')
                loader.load_string(launch_str, config)
            except roslaunch.xmlloader.XmlParseException as e:
                raise roslaunch.core.RLException(
                    'Launch string: %s\nException: %s' % (launch_str, e))
            except roslaunch.loader.LoadException as e:
                raise roslaunch.core.RLException(
                    'Launch string: %s\nException: %s' % (launch_str, e))
    # choose machines for the nodes
    if assign_machines:
        config.assign_machines()
    return config


# overwrite load_config_default function for kinetic
# see: https://github.com/ros/ros_comm/pull/1115
roslaunch.config.load_config_default = _load_config_default


class AppManager(object):

    def __init__(
            self, robot_name, interface_master, app_list,
            exchange, plugins=None, enable_app_replacement=True,
    ):
        self._robot_name = robot_name
        self._interface_master = interface_master
        self._app_list = app_list
        self._current_app = self._current_app_definition = None
        self._exchange = exchange
        self._plugins = plugins
        self._enable_app_replacement = enable_app_replacement
            
        rospy.loginfo("Starting app manager for %s"%self._robot_name)

        self._app_interface = self.scoped_name('application')

        # note: we publish into the application namespace
        self._status_pub = rospy.Publisher(
            self.scoped_name('application/app_status'), AppStatus,
            latch=True, queue_size=1)
        self._list_apps_pub = rospy.Publisher(
            self.scoped_name('app_list'), AppList,
            latch=True, queue_size=1)
        
        self._list_apps_srv  = rospy.Service(self.scoped_name('list_apps'),  ListApps,  self.handle_list_apps)
        self._start_app_srv = rospy.Service(self.scoped_name('start_app'), StartApp, self.handle_start_app)
        self._stop_app_srv   = rospy.Service(self.scoped_name('stop_app'),   StopApp,   self.handle_stop_app)
        self._reload_app_list_srv = rospy.Service(self.scoped_name('reload_app_list'), Empty, self.handle_reload_app_list)
        if (self._exchange):
            self._exchange_list_apps_pub = rospy.Publisher(self.scoped_name('exchange_app_list'), AppInstallationState, latch=True)
            self._list_exchange_apps_srv = rospy.Service(self.scoped_name('list_exchange_apps'), GetInstallationState, self.handle_list_exchange_apps)
            self._get_app_details_srv = rospy.Service(self.scoped_name('get_app_details'), GetAppDetails, self.handle_get_app_details)
            self._install_app_srv = rospy.Service(self.scoped_name('install_app'), InstallApp, self.handle_install_app)
            self._uninstall_app_srv = rospy.Service(self.scoped_name('uninstall_app'), UninstallApp, self.handle_uninstall_app)
        
            pub_names = [x.resolved_name for x in [self._list_apps_pub, self._status_pub, self._exchange_list_apps_pub]]
            service_names = [x.resolved_name for x in [self._list_apps_srv, self._start_app_srv, self._stop_app_srv, self._get_app_details_srv, self._list_exchange_apps_srv, self._install_app_srv, self._uninstall_app_srv]]
        else:
            pub_names = [x.resolved_name for x in [self._list_apps_pub, self._status_pub]]
            service_names = [x.resolved_name for x in [self._list_apps_srv, self._start_app_srv, self._stop_app_srv]]
        
        self._api_sync = MasterSync(self._interface_master,
                                    local_service_names=service_names,
                                    local_pub_names=pub_names)

        self._launch = None
        self._plugin_launch = None
        self._interface_sync = None
        self._exit_code = None
        self._stopped = None
        self._stopping = None
        self._current_process = None
        self._current_plugins = None
        self._current_plugin_processes = None
        self._plugin_context = None
        self._plugin_insts = None
        self._start_time = None

        roslaunch.pmon._init_signal_handlers()

        if (self._exchange):
            self._exchange.update_local()

        # for time profiling
        # time profiling is commented out because it will slow down.
        # comment in when you debug it
        # start_time = time.time()
        # pr = cProfile.Profile()
        # pr.enable()
        self._app_list.update()
        self.publish_exchange_list_apps()
        self.publish_list_apps()
        # pr.disable()
        # s = StringIO()
        # sortby = 'cumulative'
        # ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
        # ps.print_stats()
        # print(s.getvalue())
        # end_time = time.time()
        # rospy.logerr('total time: {}'.format(end_time - start_time))

        rospy.loginfo("Initializing default launcher")
        self._default_launch = roslaunch.parent.ROSLaunchParent(
            rospy.get_param("/run_id"), [], is_core=False, show_summary=False)
        self._default_launch.start(auto_terminate=False)

    def shutdown(self):
        if self._api_sync:
            self._api_sync.stop()
        if self._interface_sync:
            self._interface_sync.stop()
        self._stopped = True
        self.__stop_current()

    def _get_current_app(self):
        return self._current_app

    def _set_current_app(self, app, app_definition):
        self._current_app = app
        self._current_app_definition = app_definition
        
        if self._list_apps_pub:
            if app is not None:
                self._list_apps_pub.publish([app], self._app_list.get_app_list())
            else:
                self._list_apps_pub.publish([], self._app_list.get_app_list())
    
    def scoped_name(self, name):
        return rosgraph.names.canonicalize_name('/%s/%s'%(self._robot_name, rospy.remap_name(name)))

    def handle_get_app_details(self, req):
        return GetAppDetailsResponse(app=self._exchange.get_app_details(req.name))
    
    def handle_list_exchange_apps(self, req):
        if (self._exchange == None):
            return None
        if (req.remote_update):
            rospy.loginfo("UPDATE")
            if (not self._exchange.update()):
                return None
        i_apps = self._exchange.get_installed_apps()
        a_apps = self._exchange.get_available_apps()
        return GetInstallationStateResponse(installed_apps=i_apps, available_apps=a_apps)

    def publish_list_apps(self):
        if self._current_app:
            self._list_apps_pub.publish([self._current_app], self._app_list.get_app_list())
        else:
            self._list_apps_pub.publish([], self._app_list.get_app_list())

    def publish_exchange_list_apps(self):
        if (self._exchange == None):
            return
        i_apps = self._exchange.get_installed_apps()
        a_apps = self._exchange.get_available_apps()
        self._exchange_list_apps_pub.publish(i_apps, a_apps)
    
    def handle_install_app(self, req):
        appname = req.name
        if (self._exchange.install_app(appname)):
            self._app_list.update()
            self.publish_list_apps()
            self.publish_exchange_list_apps()
            return InstallAppResponse(installed=True, message="app [%s] installed"%(appname))
        else:
            return InstallAppResponse(installed=False, message="app [%s] could not be installed"%(appname))

    def handle_uninstall_app(self, req):
        appname = req.name
        if (self._exchange.uninstall_app(appname)):
            self._app_list.update()
            self.publish_list_apps()
            self.publish_exchange_list_apps()
            return UninstallAppResponse(uninstalled=True, message="app [%s] uninstalled"%(appname))
        else:
            return UninstallAppResponse(uninstalled=False, message="app [%s] could not be uninstalled"%(appname))

    def handle_list_apps(self, req):
        rospy.loginfo("Listing apps")
        current = self._current_app
        if current:
            running_apps = [current]
        else:
            running_apps = []
        self._app_list.update()
        rospy.loginfo("done listing apps")
        return ListAppsResponse(running_apps=running_apps, available_apps=self._app_list.get_app_list())

    def handle_start_app(self, req):
        rospy.loginfo("start_app: %s"%(req.name))
        if self._current_app:
            if self._current_app_definition.name == req.name:
                return StartAppResponse(started=True, message="app [%s] already started"%(req.name), namespace=self._app_interface)
            elif not self._enable_app_replacement:
                return StartAppResponse(
                    started=False,
                    message="app [%s] is denied because app [%s] is already running."
                            % (req.name, self._current_app_definition.name),
                    namespace=self._app_interface,
                    error_code=StatusCodes.MULTIAPP_NOT_SUPPORTED)
            else:
                self.stop_app(self._current_app_definition.name)

        appname = req.name
        rospy.loginfo("Loading app: %s"%(appname))
        try:
            if self._app_list and self._app_list.get_app(appname):
                app = self._app_list.get_app(appname)
            else:
                app = load_AppDefinition_by_name(appname)
        except ValueError as e:
            return StartAppResponse(started=False, message=str(e), error_code=StatusCodes.BAD_REQUEST)
        except InvalidAppException as e:
            return StartAppResponse(started=False, message=str(e), error_code=StatusCodes.INTERNAL_ERROR)
        except NotFoundException as e:
            return StartAppResponse(started=False, message=str(e), error_code=StatusCodes.NOT_FOUND)

        try:
            self._set_current_app(App(name=appname), app)

            self._status_pub.publish(AppStatus(AppStatus.INFO, 'launching %s'%(app.display_name)))

            launch_files = []
            if app.launch:
                if len(req.args) == 0:
                    launch_files = [app.launch]
                    rospy.loginfo("Launching: {}".format(app.launch))
                else:
                    app_launch_args = []
                    for arg in req.args:
                        app_launch_args.append("{}:={}".format(arg.key, arg.value))
                    launch_files = [(app.launch, app_launch_args)]
                    rospy.loginfo("Launching: {} {}".format(app.launch, app_launch_args))

            plugin_launch_files = []
            if app.plugins:
                self._current_plugins = []
                if 'start_plugin_order' in app.plugin_order:
                    plugin_names = [p['name'] for p in app.plugins]
                    plugin_order = app.plugin_order['start_plugin_order']
                    if len(set(plugin_names) - set(plugin_order)) > 0:
                        rospy.logwarn(
                            "Some plugins are defined in plugins but not written in start_plugin_order: {}"
                            .format(set(plugin_names) - set(plugin_order)))
                    app_plugins = []
                    for plugin_name in plugin_order:
                        if plugin_name not in plugin_names:
                            rospy.logerr("app plugin '{}' not found in app file.".format(plugin_name))
                            continue
                        app_plugins.append(
                            app.plugins[plugin_names.index(plugin_name)])
                else:
                    app_plugins = app.plugins
                for app_plugin in app_plugins:
                    app_plugin_type = app_plugin['type']
                    try:
                        plugin = next(
                            p for p in self._plugins if p['name'] == app_plugin_type)
                        self._current_plugins.append((app_plugin, plugin))
                        if 'launch' in plugin and plugin['launch']:
                            plugin_launch_file = find_resource(plugin['launch'])
                            launch_args = {}
                            if 'launch_args' in app_plugin:
                                launch_args.update(app_plugin['launch_args'])
                            if 'launch_arg_yaml' in app_plugin:
                                with open(app_plugin['launch_arg_yaml']) as yaml_f:
                                    yaml_launch_args = yaml.load(yaml_f)
                                for k, v in yaml_launch_args.items():
                                    if k in launch_args:
                                        rospy.logwarn("'{}' is set both in launch_args and launch_arg_yaml".format(k))
                                        rospy.logwarn("'{}' is overwritten: {} -> {}".format(k, launch_args[k], v))
                                    launch_args[k] = v
                            plugin_launch_args = []
                            for k, v in launch_args.items():
                                if isinstance(v, list):
                                    v = " ".join(map(str, v))
                                plugin_launch_args.append("{}:={}".format(k, v))
                            rospy.loginfo(
                                "Launching plugin: {} {}".format(
                                    plugin_launch_file, plugin_launch_args))
                            plugin_launch_files.append(
                                (plugin_launch_file, plugin_launch_args))
                    except StopIteration:
                        rospy.logerr(
                            'There is no available app_manager plugin: {}'
                            .format(app_plugin_type))

            #TODO:XXX This is a roslaunch-caller-like abomination.  Should leverage a true roslaunch API when it exists.
            if app.launch:
                self._launch = roslaunch.parent.ROSLaunchParent(
                    rospy.get_param("/run_id"), launch_files,
                    is_core=False, process_listeners=())
            if len(plugin_launch_files) > 0:
                self._plugin_launch = roslaunch.parent.ROSLaunchParent(
                    rospy.get_param("/run_id"), plugin_launch_files,
                    is_core=False, process_listeners=())

            if self._launch:
                self._launch._load_config()
            if self._plugin_launch:
                self._plugin_launch._load_config()

            #TODO: convert to method
            nodes = []
            if self._launch:
                nodes.extend(self._launch.config.nodes)
            if app.run:
                nodes.append(app.run)
            for N in nodes:
                for t in app.interface.published_topics.keys():
                    N.remap_args.append((t, self._app_interface + '/' + t))
                for t in app.interface.subscribed_topics.keys():
                    N.remap_args.append((t, self._app_interface + '/' + t))

            # run plugin modules first
            self._current_plugin_processes = []
            if self._current_plugins:
                self._plugin_context = {}
                self._plugin_insts = {}
                for app_plugin, plugin in self._current_plugins:
                    if 'module' in plugin and plugin['module']:
                        plugin_args = {}
                        start_plugin_args = {}
                        if 'plugin_args' in app_plugin:
                            plugin_args.update(app_plugin['plugin_args'])
                        if 'plugin_arg_yaml' in app_plugin:
                            with open(app_plugin['plugin_arg_yaml']) as yaml_f:
                                yaml_plugin_args = yaml.load(yaml_f)
                            for k, v in yaml_plugin_args.items():
                                if k in plugin_args:
                                    rospy.logwarn("'{}' is set both in plugin_args and plugin_arg_yaml".format(k))
                                    rospy.logwarn("'{}' is overwritten: {} -> {}".format(k, plugin_args[k], v))
                                plugin_args[k] = v
                        if 'start_plugin_args' in app_plugin:
                            start_plugin_args.update(app_plugin['start_plugin_args'])
                        if 'start_plugin_arg_yaml' in app_plugin:
                            with open(app_plugin['start_plugin_arg_yaml']) as yaml_f:
                                yaml_plugin_args = yaml.load(yaml_f)
                            for k, v in yaml_plugin_args.items():
                                if k in start_plugin_args:
                                    rospy.logwarn("'{}' is set both in start_plugin_args and start_plugin_arg_yaml".format(k))
                                    rospy.logwarn("'{}' is overwritten: {} -> {}".format(k, start_plugin_args[k], v))
                                start_plugin_args[k] = v
                        plugin_args.update(start_plugin_args)
                        mod = __import__(plugin['module'].split('.')[0])
                        for sub_mod in plugin['module'].split('.')[1:]:
                            mod = getattr(mod, sub_mod)
                        plugin_inst = mod()
                        plugin_inst.app_manager_start_plugin(
                            app, self._plugin_context, plugin_args)
                        self._plugin_insts[plugin['module']] = plugin_inst
                    if 'run' in plugin and plugin['run']:
                        p, a = roslib.names.package_resource_name(plugin['run'])
                        args = plugin.get('run_args', None)
                        node = roslaunch.core.Node(p, a, args=args, output='screen')
                        proc, success = self._default_launch.runner.launch_node(node)
                        if not success:
                            raise roslaunch.core.RLException(
                                "failed to launch plugin %s/%s"%(node.package, node.type))
                        self._current_plugin_processes.append(proc)

            # then, start plugin launches
            if self._plugin_launch:
                self._plugin_launch.start()

            # finally launch main launch
            if self._launch:
                self._launch.start()
            if app.run:
                node = app.run
                proc, success = self._default_launch.runner.launch_node(node)
                if not success:
                    raise roslaunch.core.RLException(
                        "failed to launch %s/%s"%(node.package, node.type))
                self._current_process = proc

            if app.timeout is not None:
                self._start_time = rospy.Time.now()

            fp = [self._app_interface + '/' + x for x in app.interface.subscribed_topics.keys()]
            lp = [self._app_interface + '/' + x for x in app.interface.published_topics.keys()]

            self._interface_sync = MasterSync(self._interface_master, foreign_pub_names=fp, local_pub_names=lp)

            thread.start_new_thread(self.app_monitor, (app.launch,))

            return StartAppResponse(started=True, message="app [%s] started"%(appname), namespace=self._app_interface)
        
        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            try:
                # attempt to kill any launched resources
                self._stop_current()
            except:
                pass
            finally:
                self._set_current_app(None, None)
            self._status_pub.publish(AppStatus(AppStatus.INFO, 'app start failed'))
            rospy.logerr(
                "app start failed [{}, line {}: {}]".format(fname, exc_tb.tb_lineno, str(e)))
            return StartAppResponse(started=False, message="internal error [%s, line %d: %s]"%(fname, exc_tb.tb_lineno, str(e)), error_code=StatusCodes.INTERNAL_ERROR)
    
    def _stop_current(self):
        try:
            self._stopping = True
            self.__stop_current()
        finally:
            self._launch = None
            self._plugin_launch = None
            self._exit_code = None
            self._stopped = None
            self._stopping = None
            self._current_process = None
            self._current_plugins = None
            self._current_plugin_processes = None
            self._plugin_context = None
            self._plugin_insts = None
            self._start_time = None
        try:
            self._interface_sync.stop()
        finally:
            self._interface_sync = None

    def __stop_current(self):
        if self._api_sync:
            self._api_sync.stop()
        if self._launch:
            self._launch.shutdown()
            if (self._exit_code is None
                    and self._launch.pm
                    and len(self._launch.pm.dead_list) > 0):
                exit_codes = [p.exit_code for p in self._launch.pm.dead_list]
                self._exit_code = max(exit_codes)
        if self._current_process:
            self._current_process.stop()
            if (self._exit_code is None
                    and self._default_launch.pm
                    and len(self._default_launch.pm.dead_list) > 0):
                self._exit_code = self._default_launch.pm.dead_list[0].exit_code
        if not self._exit_code is None and self._exit_code > 0:
            rospy.logerr(
                "App stopped with exit code: {}".format(self._exit_code))
        if self._plugin_launch:
            self._plugin_launch.shutdown()
        if self._current_plugin_processes:
            for p in self._current_plugin_processes:
                p.stop()
        if self._current_plugins:
            self._plugin_context['exit_code'] = self._exit_code
            self._plugin_context['stopped'] = self._stopped
            if 'stop_plugin_order' in self._current_app_definition.plugin_order:
                plugin_names = [p['name'] for p in self._current_app_definition.plugins]
                plugin_order = self._current_app_definition.plugin_order['stop_plugin_order']
                if len(set(plugin_names) - set(plugin_order)) > 0:
                    rospy.logwarn(
                        "Some plugins are defined in plugins but not written in stop_plugin_order: {}"
                        .format(set(plugin_names) - set(plugin_order)))
                current_plugins = []
                for plugin_name in plugin_order:
                    if plugin_name not in plugin_names:
                        rospy.logerr("app plugin '{}' not found in app file.".format(plugin_name))
                        continue
                    current_plugin_names = [p['name'] for p, _ in self._current_plugins]
                    if plugin_name not in current_plugin_names:
                        rospy.logwarn("app plugin '{}' is not running, so skip stopping".format(plugin_name))
                        continue
                    current_plugins.append(
                        self._current_plugins[current_plugin_names.index(plugin_name)])
            else:
                current_plugins = self._current_plugins
            for app_plugin, plugin in current_plugins:
                if 'module' in plugin and plugin['module']:
                    plugin_args = {}
                    stop_plugin_args = {}
                    if 'plugin_args' in app_plugin:
                        plugin_args.update(app_plugin['plugin_args'])
                    if 'plugin_arg_yaml' in app_plugin:
                        with open(app_plugin['plugin_arg_yaml']) as yaml_f:
                            yaml_plugin_args = yaml.load(yaml_f)
                        for k, v in yaml_plugin_args.items():
                            if k in plugin_args:
                                rospy.logwarn("'{}' is set both in plugin_args and plugin_arg_yaml".format(k))
                                rospy.logwarn("'{}' is overwritten: {} -> {}".format(k, plugin_args[k], v))
                            plugin_args[k] = v
                    if 'stop_plugin_args' in app_plugin:
                        stop_plugin_args.update(app_plugin['stop_plugin_args'])
                    if 'stop_plugin_arg_yaml' in app_plugin:
                        with open(app_plugin['stop_plugin_arg_yaml']) as yaml_f:
                            yaml_plugin_args = yaml.load(yaml_f)
                        for k, v in yaml_plugin_args.items():
                            if k in stop_plugin_args:
                                rospy.logwarn("'{}' is set both in stop_plugin_args and stop_plugin_arg_yaml".format(k))
                                rospy.logwarn("'{}' is overwritten: {} -> {}".format(k, stop_plugin_args[k], v))
                            stop_plugin_args[k] = v
                    plugin_args.update(stop_plugin_args)
                    if plugin['module'] in self._plugin_insts:
                        plugin_inst = self._plugin_insts[plugin['module']]
                    else:
                        mod = __import__(plugin['module'].split('.')[0])
                        for sub_mod in plugin['module'].split('.')[1:]:
                            mod = getattr(mod, sub_mod)
                        plugin_inst = mod()
                    plugin_inst.app_manager_stop_plugin(
                        self._current_app_definition,
                        self._plugin_context, plugin_args)

    def handle_stop_app(self, req):
        rospy.loginfo("handle stop app: %s"%(req.name))
        self._stopped = True
        return self.stop_app(req.name)

    def handle_reload_app_list(self, req=None):
        try:
            self._app_list.update()
            self.publish_list_apps()
            self.publish_exchange_list_apps()
            rospy.loginfo("app list is reloaded")
        except Exception as e:
            rospy.logerr("Failed to reload app list: %s" % e)
        return EmptyResponse()

    def app_monitor(self, is_launch):
        def get_target():
            if is_launch:
                return self._launch
            return self._current_process
        def is_done(target):
            if is_launch:
                return (not target.pm or target.pm.done)
            return target.stopped

        while get_target():
            time.sleep(0.1)
            target = get_target()
            timeout = self._current_app_definition.timeout
            appname = self._current_app_definition.name
            now = rospy.Time.now()
            if target:
                if is_done(target):
                    time.sleep(1.0)
                    if not self._stopping:
                        self.stop_app(appname)
                    break
                if (timeout is not None and
                        self._start_time is not None and
                        (now - self._start_time).to_sec() > timeout):
                    self._stopped = True
                    self.stop_app(appname)
                    rospy.logerr(
                        'app {} is stopped because of timeout: {}s'.format(
                            appname, timeout))
                    break



    def stop_app(self, appname):
        resp = StopAppResponse(stopped=False)
        try:
            app = self._current_app_definition

            # request to stop all apps.
            if app is not None and appname == '*':
                appname = app.name

            if app is None or app.name != appname:
                rospy.loginfo("handle stop app: app [%s] is not running [x]"%(appname))
                resp.error_code = StatusCodes.NOT_RUNNING
                resp.message = "app %s is not running"%(appname)                    
            else:
                try:
                    if self._launch or self._current_process:
                        rospy.loginfo("handle stop app: stopping app [%s]"%(appname))
                        self._status_pub.publish(AppStatus(AppStatus.INFO, 'stopping %s'%(app.display_name)))
                        self._stop_current()
                        rospy.loginfo("handle stop app: app [%s] stopped"%(appname))
                        resp.stopped = True
                        resp.message = "%s stopped"%(appname)
                    else:
                        rospy.loginfo("handle stop app: app [%s] is not running"%(appname))
                        resp.message = "app [%s] is not running"%(appname)
                        resp.error_code = StatusCodes.NOT_RUNNING
                finally:
                    self._launch = None
                    self._current_process = None
                    self._set_current_app(None, None)

        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            rospy.logerr("handle stop app: internal error [%s, line %d: %s]"%(fname, exc_tb.tb_lineno, str(e)))
            resp.error_code = StatusCodes.INTERNAL_ERROR
            resp.message = "internal error: %s"%(str(e))
            
        return resp 
