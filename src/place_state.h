#pragma once

enum class PlaceState {
  MoveToPreparePose,
  MoveToPreparePoseFailed,
  Approach,
  ApproachFailed,
  OpenGripper,
  OpenGripperFailed,
  Retreat,
  RetreatFailed,
  Completed
};
