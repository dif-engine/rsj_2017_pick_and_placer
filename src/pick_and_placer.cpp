// Copyright 2017 Geoffrey Biggs (geoffrey.biggs@aist.go.jp)

#include <ros/ros.h>
#include <string>
#include <vector>

#include "pick_and_placer.h"
#include "pick_state.h"
#include "place_state.h"

#define USE_DO_MOVE_VERTICAL 0

PickNPlacer::PickNPlacer(Arm& arm, Logger& logger, PlanningScene& scene)
    : arm_(arm), logger_(logger), scene_(scene) {
      Initialize();
}

void PickNPlacer::Initialize() {
  // Specify end-effector positions in the configured task frame
  arm_.Initialize();

  // Initialise the planning scene with known objects
  SetupPlanningScene();

  // Start by moving to the vertical pose
  #if USE_DO_MOVE_VERTICAL
  arm_.DoMoveVertical();
  #endif 
}

void PickNPlacer::DoMoveVertical() {
  #if USE_DO_MOVE_VERTICAL
  arm_.DoMoveVertical();
  #endif 
}

bool PickNPlacer::DoPickAndPlace(double x, double y) {
  if (DoPick(x, y)) {
    return DoPlace();
  }
}

bool PickNPlacer::DoPick(double x, double y) {
  // Add the newly-detected object
  scene_.AddBox(x, y);
  // Sleep a little to let the messages flow and be processed
  ros::Duration(sleepTime).sleep();

  PickState pick = arm_.DoPick(x, y);
  return pick == PickState::Completed;
}

bool PickNPlacer::DoPlace() {
  PlaceState place = arm_.DoPlace();
  // Remove the object now that we don't care about it any more
  scene_.RemoveBox();
  return place == PlaceState::Completed;
}

void PickNPlacer::SetupPlanningScene() {
  scene_.Initialize();
}
