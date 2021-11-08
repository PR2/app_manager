^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package app_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2021-11-08)
------------------
* update setuptools to follow noetic migration guide (`#36 <https://github.com/pr2/app_manager/issues/36>`_)
* app_manager cannot start app after failing app #42  (`#42 <https://github.com/pr2/app_manager/issues/42>`_)

  * set current_app None when start failed
  * add test_start_fail.test
  * need catch error on _stop_current()
  * add test_start_fail.test
    test to check `#42 <https://github.com/pr2/app_manager/issues/42>`_, app_manager cannot start app after failing app

* add test to check if we forget catkin_install_python (`#44 <https://github.com/pr2/app_manager/issues/44>`_)

  * call app_manager/appA with python2/python3 with ROS_PYTHON_VERSION
  * use catkin_install_python for noetic
  * add test to check if we forget to use catkin_install_python

* add_rostest(test/test_plugin.test) (`#45 <https://github.com/pr2/app_manager/issues/45>`_)

  * run rosdep install in devel_create_tasks.Dockerfile
  * update to format3 and install python-rosdep
  * use port 11313 for app_manager in test_plugin.test
  * add_rostest(test/test_plugin.test)

* add more test code (`#41 <https://github.com/pr2/app_manager/issues/41>`_

  * show more error messages
  * default return value of plugin_order must be list
  * plugins: 'launch_args', 'plugin_args', 'start_plugin_args', 'stop_plugin_args' must be dict, not list
  * test_plugin: add test to check plugins
  * use list(self.subs.items()) instead of self.subs.items()
  * Error processing request: '>' not supported between instances of 'NoneType' and 'int'
  * python3: AttributeError: 'dict' object has no attribute 'iteritems'
  * add test for list_apps/stop_app, add test_stop_app.py
  * python3: AttributeError: 'dict' object has no attribute 'iterkeys'
  * add 2to3 in CHECK_PYTHON3_COMPILE
  * add test/test_app.test
  * test/resources/example-moin.launch: use arg launch_prefox to select if we use xterm or not

* add arguments in StartAppRequest (`#27 <https://github.com/pr2/app_manager/issues/27>`_)

  * use req.args for launch args in app_manager.py
  * add args in StartApp srv

* do not run stop_app when _stopping is true (`#38 <https://github.com/pr2/app_manager/issues/38>`_)
* fix travis build (`#39 <https://github.com/pr2/app_manager/issues/39>`_)

  * fix typo in .travis.yml
  * run with full path
  * add CHECK_PYTHON2_COMPILE and CHECK_PYTHON3_COMPILE tests

* use plugins as instance / use normal method in app_manager_plugin (`#37 <https://github.com/pr2/app_manager/issues/37>`_)
* set stopped true in app timeout (`#31 <https://github.com/pr2/app_manager/issues/31>`_)
* use system python to check python3 compileall (`#34 <https://github.com/pr2/app_manager/issues/34>`_)

* Contributors: Kei Okada, Shingo Kitagawa

1.2.0 (2021-03-03)
------------------
* Merge pull request `#29 <https://github.com/pr2/app_manager/issues/29>`_ from knorth55/add-stopped
* Merge pull request `#28 <https://github.com/pr2/app_manager/issues/28>`_ from knorth55/add-timeout
* Merge pull request `#30 <https://github.com/pr2/app_manager/issues/30>`_ from knorth55/add-all-availables
  add available_apps in all platform apps
* add available_apps in all platform apps
* add stopped context in app_manager plugin
* add timeout field in app file
* add enable_app_replacement param in app_manager (`#26 <https://github.com/pr2/app_manager/issues/26>`_)
  This allows for both behaviors (replace currently running app or error out) and let's users choose without changing code...
* add noetic test and also checks python 2/3 compatibility (`#24 <https://github.com/pr2/app_manager/issues/24>`_)
  * add noetic test and also checks python 2/3 compatibility
  * fix to support both python 2/3
  * use rospy.log** instead of print
  Co-authored-by: Shingo Kitagawa <shingogo@hotmail.co.jp>
* Add app manager plugin (`#25 <https://github.com/pr2/app_manager/issues/25>`_)
  Enable aspect-oriented modelling, e.g.
  - send a mail when someone starts an app
  - auto-record rosbags during app run
  - auto-upload files on app-close
  ...
  * start plugin launch in app_manager
  * start and stop plugin function
  * pass app in plugin functions
  * add exit_code in stop_plugin_attr
  * stop plugin functions when shutdown is called
  * launch when plugin
  * load plugin launch when it exists
  * use ctx instead of exit_code
  * pass exit_code to ctx
  * add plugins in AppDefinition
  * add _current_plugins and _plugin_context
  * pass launch arguments
  * add plugin_args for app plugins
  * overwrite roslaunch.config.load_config_default for kinetic
  * add __stop_current for shutdown and __stop_current
  * support "module: null" syntax for app definition
  * add app_manager plugin base class
  * refactor scripts/app_manager
  * add start_plugin_args and stop_plugin_args
  * add start_plugin_arg_yaml and stop_plugin_arg_yaml
  * add launch_arg_yaml
  * add plugin_order to set plugin order
  * update readme to add plugin doc
  * update readme to add app definitions
* fix readme (`#23 <https://github.com/pr2/app_manager/issues/23>`_)
* add exit_code log in app_manager (`#22 <https://github.com/pr2/app_manager/issues/22>`_)
  add exit_code log in app_manager
  During successful execution, `dead_list` should always end up empty.
* add all platform for all robots (`#17 <https://github.com/pr2/app_manager/issues/17>`_)
  add an additional keyword to explicitly support 'all' platforms
* use rospack to search for app_manager app_dir (`#19 <https://github.com/pr2/app_manager/issues/19>`_)
  * use rospack to search for app_manager app_dir
  * remove unused imports
  * use both --applist and plugin app_dir
* Merge pull request `#20 <https://github.com/pr2/app_manager/issues/20>`_ from knorth55/fix-print-python3
  use rospy.logerr to escape print error in python3
* use rospy.logerr to escape print error in python3
* Contributors: Kei Okada, Michael Görner, Shingo Kitagawa

1.1.1 (2020-04-13)
------------------
* use python3.5 for travis (`#18 <https://github.com/pr2/app_manager/issues/18>`_)
* use app_manager/example-min (`#15 <https://github.com/pr2/app_manager/issues/15>`_)

  * example-min.launch location changed due to https://github.com/PR2/app_manager/pull/15
  * use app_manager/example-min, since talker.py was removed from rospy in melodic https://github.com/ros/ros_comm/pull/1847

* update document (`#14 <https://github.com/pr2/app_manager/issues/14>`_)

  * Add more info for how to start apps, closes https://github.com/PR2/app_manager/issues/13

* Fix travis (`#16 <https://github.com/pr2/app_manager/issues/16>`_)

  * fix workspace name due to `ros-infrastructure/ros_buildfarm#577 <https://github.com/ros-infrastructure/ros_buildfarm/issues/577>`_


* install launch directory (`#9 <https://github.com/pr2/app_manager/issues/9>`_)
* add run_depend of app_manager in README.md (`#11 <https://github.com/pr2/app_manager/issues/11>`_)
* update travis.yml (`#10 <https://github.com/pr2/app_manager/issues/10>`_)

  * add empty applist0 directory
    c.f. https://stackoverflow.com/questions/115983/how-can-i-add-an-empty-directory-to-a-git-repository
  * calling self._load() and updating self._file_mtime are never happens, since self.update() is removed in https://github.com/PR2/app_manager/pull/7/files#diff-a8d7b30ba0e424e10aa794dec1928181L98
  * revert code from `#7 <https://github.com/pr2/app_manager/issues/7>`_, which wrongly removed invalid_installed_files
  * example-min.launch file has been moved to subdir since 2012
     https://github.com/ros/ros_comm/commit/964da45c6959bf9c2bde8680c69d1ab36e3770b1#diff-03b2e74d781fea8d7240c1fdd29a41a9

* Contributors: Kei Okada, Shingo Kitagawa, Takayuki Murooka, Yuki Furuta

1.1.0 (2018-08-29)
------------------
* Support loading installed apps from export tags (`#7 <https://github.com/PR2/app_manager//issues/7>`_)
  * app_manager: add reload_app_list service to dynamically reload apps
  * filter apps by robot platform
  * add support for loading app directories from plugins
* Cleanup unused files (`#6 <https://github.com/PR2/app_manager//issues/6>`_)
* Contributors: Yuki Furuta

1.0.5 (2018-02-14)
------------------
* Merge pull request `#5 <https://github.com/pr2/app_manager/issues/5>`_ from k-okada/orp
  change maintainer to ROS orphaned package maintainer
* change maintainer to ROS orphaned package maintainer
* Contributors: Kei Okada

1.0.3 (2015-02-06)
------------------

1.0.2 (2014-10-14)
------------------
* changelogs
* Fixed installs on app_manager
* Contributors: TheDash

* Fixed installs on app_manager
* Contributors: TheDash
