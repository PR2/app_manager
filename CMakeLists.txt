cmake_minimum_required(VERSION 2.8.3)
project(app_manager)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  rosgraph
  roslaunch
  rospy
  rosunit
  std_msgs
  )

catkin_python_setup()

add_message_files(
    FILES
    AppList.msg
    ClientApp.msg
    AppInstallationState.msg
    App.msg
    KeyValue.msg
    AppStatus.msg
    ExchangeApp.msg
    Icon.msg
    StatusCodes.msg
)
add_service_files(
    FILES
    GetAppDetails.srv
    ListApps.srv
    UninstallApp.srv
    InstallApp.srv
    GetInstallationState.srv
    StartApp.srv
    StopApp.srv
)


generate_messages(
    DEPENDENCIES std_msgs
)

catkin_package(
    CATKIN_DEPENDS rospy roslaunch rosgraph rosunit
)


install(PROGRAMS bin/rosget bin/appmaster
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch scripts test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_app.py)
  catkin_add_nosetests(test/test_app_list.py)
endif()
