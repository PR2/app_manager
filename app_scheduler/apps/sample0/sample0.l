#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")

(ros::roseus "sample0")
(ros::ros-info "sample0 start")
(ros::ros-info "publishing /app_scheduler/sample0 ...")
(one-shot-publish "/app_scheduler/sample0"
                  (instance std_msgs::String :init :data "sample0"))
(ros::ros-info "sample0 finish")
(exit)
