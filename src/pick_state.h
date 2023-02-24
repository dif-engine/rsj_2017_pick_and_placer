#pragma once

enum class PickState {
  MoveToPreparePose,
  MoveToPreparePoseFailed,
  OpenGripper,
  OpenGripperFailed,
  Approach,
  ApproachFailed,
  Grasp,
  GraspFailed,
  Retreat,
  RetreatFailed,
  Completed
};

