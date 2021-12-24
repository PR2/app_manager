#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")

(ros::roseus "sample1")
(ros::ros-info "sample1 start")
(ros::ros-info "publishing /app_scheduler/sample1 ...")
(one-shot-publish "/app_scheduler/sample1"
                  (instance std_msgs::String :init :data "sample1"))
(ros::ros-info "sample1 finish")
(exit)
