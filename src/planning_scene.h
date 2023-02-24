#pragma once

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "arm.h"
#include "logger.h"

class PlanningScene {
public:
  PlanningScene(Arm& arm, Logger& logger);
  virtual void Initialize();
  virtual void AddBox(double x, double y);
  virtual void RemoveBox();

protected:
  virtual void AddTable();
  moveit::planning_interface::PlanningSceneInterface scene_;
  Arm& arm_;
  Logger& logger_;
};
