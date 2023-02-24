#pragma once

#include <string>

struct PickNPlacerParams {
  std::string scene_task_frame_;
  float place_x_;
  float place_y_;
  float pick_prepare_z_;
  float pick_z_;
  float place_prepare_z_;
  float place_z_;
  float gripper_open_;
  float gripper_close_;
};
