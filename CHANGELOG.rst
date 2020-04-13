^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package app_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
