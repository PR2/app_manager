app_manager [![Build Status](https://travis-ci.com/PR2/app_manager.svg?branch=kinetic-devel)](https://travis-ci.org/PR2/app_manager)
====================================================================================================================================

A package for making launch file an application

## Installation

Run `sudo apt-get install ros-$ROS_DISTRO-app-manager`

## Usage

The `app_manager` node loads information of available application from `.installed` files.
`.installed` file is a yaml file that defines installed applications in a package like below:

```yaml
# package_root/apps/app.installed
apps:
- app: pkg_name/app_name1
  display: sample app
- app: pkg_name/app_name2
  display: another sample app
```

Once `.installed` file is defined, you have to notify the location of the files to `app_manager` by either of two ways:

1. Give the locations as arguments

One way to notify the location is to add `--applist` argument with `rosrun`.

```bash
rosrun app_manager app_manager --applist `rospack find package_root`/apps
```

This is useful for testing one small `.installed` file or a demonstration.

2. Register as export attributes

Another way to notify the location is to define them in `<export>` tag in `package.xml`.

```xml
<!-- package_root/package.xml -->
<package>
  ...
  <run_depend>app_manager</run_depend>
  ...
  <export>
    <app_manager app_dir="${prefix}/apps"/>
  </export>
</package>
```

And launch `app_manager` without any argument:

```bash
rosrun app_manager app_manager
```

`app_manager` node automatically searches all `.installed` files and register as available applications.

Applications can be filtered by platform defined in each `.app` file.
If you set the parameter `/robot/type` to `pr2`, then apps for platform `pr2` will be available.

```bash
rosparam set /robot/type pr2
```


## APIs

All topics/services are advertised under the namespace specified by the parameter `/robot/name`.

### Publishing Topics

- `app_list`: List available/running applications
- `application/app_status`: Current status of app manager

### Services

- `list_apps`: List available/running applications
- `start_app`: Start an available application
- `stop_app`: Stop a runniing application
- `reload_app_list`: Reload installed applications from `*.installed`) file.


## Examples

Start default roscore
```
$ roscore

```

and start another roscore for app_manager from another Terminal

```
$ roscore -p 11312
```

Start app_manager
```
$ rosrun app_manager app_manager --applist `rospack find app_manager`/test/applist1 _interface_master:=http://localhost:11312
```
Make sure that it founds the apps
```
[INFO] [1575604033.724035]: 1 apps found in /home/user/catkin_ws/src/app_manager/test/applist1/apps1.installed
```

Use service calls to list and start apps.
```
$ rosservice call robot/list_apps
running_apps: []
available_apps:
  -
    name: "app_manager/appA"
    display_name: "Android Joystick"
    icon:
      format: ''
      data: []
    client_apps: []
$ rosservice call /robot/start_app app_manager/appA
started: True
error_code: 0
message: "app [app_manager/appA] started"
namespace: "/robot/application"
```

## Plugins

You can define `app_manager` plugins as below in app file such as `test.app`.

```yaml
plugins: # plugin definitions
  - name: mail_notifier_plugin  # name to identify this plugin
    type: app_notifier/mail_notifier_plugin  # plugin type
    launch_args:  # arguments for plugin launch file
      - foo: hello
    launch_arg_yaml: /etc/mail_notifier_launch_arg.yaml  # argument yaml file for plugin launch file
    # in this case, these arguments will be passed.
    # {"hoge": 100, "fuga": 30, "bar": 10} will be passed to start plugin
    # {"hoge": 50, "fuga": 30} will be passed to stop plugin
    plugin_args:  # arguments for plugin function
      - hoge: 10
      - fuga: 30
    start_plugin_args:  # arguments for start plugin function
      - hoge: 100  # arguments for start plugin function arguments (it overwrites plugin_args hoge: 10 -> 100)
      - bar: 10
    stop_plugin_args:  # arguments for stop plugin function
      - hoge: 50  # arguments for stop plugin function arguments (it overwrites plugin_args hoge: 10 -> 50)
    plugin_arg_yaml: /etc/mail_notifier_plugin_arg.yaml  # argument yaml file for plugin function arguments
  - name: rosbag_recorder_plugin  # another plugin
    type app_recorder/rosbag_recorder_plugin
    launch_args:
      rosbag_path: /tmp
      rosbag_title: test.bag
      compress: true
      rosbag_topic_names:
        - /rosout
        - /tf
        - /tf_static
plugin_order: # plugin running orders. if you don't set field, plugin will be run in order in plugins field
  start_plugin_order:  # start plugin running order
    - rosbag_recorder_plugin  # 1st plugin name
    - mail_notifier_plugin  #2nd plugin name
  stop_plugin_order:  # start plugin running order
    - rosbag_recorder_plugin
    - mail_notifier_plugin
```

Sample plugin repository is [knorth55/app_manager_utils](https://github.com/knorth55/app_manager_utils).

For more detailed information, please read [#25](https://github.com/PR2/app_manager/pull/25).

## Maintainer

Yuki Furuta <<furushchev@jsk.imi.i.u-tokyo.ac.jp>>
