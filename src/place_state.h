#ifndef RSJ_2017_PICK_AND_PLACER_PLACE_STATE_H
#define RSJ_2017_PICK_AND_PLACER_PLACE_STATE_H

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

#endif //RSJ_2017_PICK_AND_PLACER_PLACE_STATE_H
