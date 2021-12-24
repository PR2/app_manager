#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")

(ros::roseus "sample2")
(ros::ros-info "sample2 start")
(ros::ros-info "publishing /app_scheduler/sample2 ...")
(one-shot-publish "/app_scheduler/sample2"
                  (instance std_msgs::String :init :data "sample2"))
(ros::ros-info "sample2 finish")
(exit)
