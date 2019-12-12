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
and start anther roscore for app_manager from another Terminal
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


## Maintainer

Yuki Furuta <<furushchev@jsk.imi.i.u-tokyo.ac.jp>>
