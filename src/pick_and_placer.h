#pragma once 

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose2D.h>//変更の必要あるね
#include "pick_and_placer_params.h"
#include "arm.h"
#include "gripper.h"
#include "logger.h"
#include "planning_scene.h"

class PickNPlacer {
public:
  PickNPlacer(Arm& arm, Logger& logger, PlanningScene& scene);
  void Initialize();
  void DoMoveVertical();
  bool DoPickAndPlace(double x, double y);
  bool DoPick(double x, double y);
  bool DoPlace();
  void SetupPlanningScene();
  int sleepTime = 1; // public so that for example tests can skip sleeping
private:
  Arm& arm_;
  Logger& logger_;
  PlanningScene& scene_;
};
