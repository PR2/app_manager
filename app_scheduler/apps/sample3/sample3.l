#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")

(ros::roseus "sample3")
(setq input (ros::get-param "~input" "default"))
(ros::ros-info "sample3 start")
(ros::ros-info "input: ~A" input)
(ros::ros-info "publishing /app_scheduler/sample3 ...")
(one-shot-publish "/app_scheduler/sample3"
                  (instance std_msgs::String :init :data input))
(ros::ros-info "sample3 finish")
(exit)
