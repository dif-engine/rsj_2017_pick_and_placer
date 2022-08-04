#ifndef RSJ_2017_PICK_AND_PLACER_GET_ROS_PARAMS_H
#define RSJ_2017_PICK_AND_PLACER_GET_ROS_PARAMS_H

#include <ros/ros.h>
#include "pick_and_placer_params.h"

void GetROSParams(PickNPlacerParams& params) {
  // Get the value for the configurable values from the parameter server, and
  // set sensible defaults for those values not specified on the parameter
  // server
  ros::param::param<float>(
    "place_x",
    params.place_x_,
    0.1);
  ros::param::param<float>("place_y", params.place_y_, -0.2);
  ros::param::param<std::string>(
    "~task_frame",
    params.scene_task_frame_,
    "base_link");
  ros::param::param<float>("~pick_prepare_z", params.pick_prepare_z_, 0.15);
  ros::param::param<float>("~pick_z", params.pick_z_, 0.05);
  ros::param::param<float>("~place_prepare_z", params.place_prepare_z_, 0.15);
  ros::param::param<float>("~place_z", params.place_z_, 0.05);
  ros::param::param<float>("~gripper_open", params.gripper_open_, 0.1);
  ros::param::param<float>("~gripper_close", params.gripper_close_, 0.025);
}

#endif //RSJ_2017_PICK_AND_PLACER_GET_ROS_PARAMS_H
