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


catkin_install_python(PROGRAMS bin/rosget bin/appmaster
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN *.py EXCLUDE)
file(GLOB_RECURSE TEST_FILES
  RELATIVE "${PROJECT_SOURCE_DIR}"
  "test/*.py")
foreach(TEST_FILE ${TEST_FILES})
  get_filename_component(DIR ${TEST_FILE} DIRECTORY)
  catkin_install_python(PROGRAMS ${TEST_FILE}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${DIR})
endforeach()
file(GLOB SCRIPTS_FILES
  RELATIVE "${PROJECT_SOURCE_DIR}"
  "${PROJECT_SOURCE_DIR}/scripts/*")
catkin_install_python(PROGRAMS ${SCRIPTS_FILES}
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  catkin_add_nosetests(test/test_app.py)
  catkin_add_nosetests(test/test_app_list.py)
  add_rostest(test/test_app.test)
  add_rostest(test/test_plugin.test)
  add_rostest(test/test_start_fail.test)
endif()
