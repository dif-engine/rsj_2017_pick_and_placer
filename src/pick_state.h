#ifndef RSJ_2017_PICK_AND_PLACER_PICK_STATE_H
#define RSJ_2017_PICK_AND_PLACER_PICK_STATE_H

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

#endif //RSJ_2017_PICK_AND_PLACER_PICK_STATE_H
