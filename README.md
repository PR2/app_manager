app_manager
===========

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
rosrun app_manager app_manager --applist package_root/apps
```

This is useful for testing one small `.installed` file or a demonstration.

2. Register as export attributes

Another way to notify the location is to define them in `<export>` tag in `package.xml`.

```xml
<!-- package_root/package.xml -->
<package>
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


## Maintainer

Yuki Furuta <<furushchev@jsk.imi.i.u-tokyo.ac.jp>>
