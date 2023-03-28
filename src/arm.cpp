#include "arm.h"

Arm::Arm(Gripper& gripper
, Logger& logger
, const std::string& group
, const std::string& gripperGroup
, PickNPlacerParams& params)
        : arm_(group, std::shared_ptr<tf2_ros::Buffer>(), ros::Duration(10))
        , logger_(logger)
        , gripper_(gripper)
        , gripper_group_(gripperGroup, std::shared_ptr<tf2_ros::Buffer>(), ros::Duration(10))
        , params_(params) {
  ROS_INFO("Setting pose reference frame");
  arm_.setPoseReferenceFrame(params_.scene_task_frame_);
  ROS_INFO("Arm initialized");
}

void Arm::Initialize() {
  gripper_.waitForServer();
  control_msgs::GripperCommandGoal goal;
  this->DoOpenGripper(goal);
  if (picking_target_attached_) {
    arm_.detachObject("pick-target");
    picking_target_attached_ = false;
  }
}

void Arm::SetParams(PickNPlacerParams& params) {
  params_ = params;
}

PickState Arm::DoPick(double x, double y) {
  // Prepare
  logger_.INFO("Moving to prepare pose");
  PickState state(PickState::MoveToPreparePose);
  geometry_msgs::PoseStamped pose;
  if (!DoPickPrepare(pose, x, y)) {
    logger_.WARN("Could not move to prepare pose");
    state = PickState::MoveToPreparePoseFailed;
    return state;
  }

  logger_.INFO("Opening gripper");
  state = PickState::OpenGripper;
  control_msgs::GripperCommandGoal goal;

  if (!DoOpenGripper(goal)) {
    logger_.WARN("Gripper open action did not complete");
    state = PickState::OpenGripperFailed;
    return state;
  }

  logger_.INFO("Executing approach");
  state = PickState::Approach;
  if (!DoApproach(pose)) {
    logger_.WARN("Could not move to grasp pose");
    state = PickState::ApproachFailed;
    return state;
  }

  logger_.INFO("Grasping object");
  state = PickState::Grasp;
  if (!DoGrasp(goal)) {
    logger_.WARN("Gripper close action did not complete");
    state = PickState::GraspFailed;
    return state;
  }

  logger_.INFO("Retreating");
  state = PickState::Retreat;
  if (!DoRetreat(pose)) {
    logger_.WARN("Could not move to retreat pose");
    state = PickState::RetreatFailed;
    return state;
  }

  logger_.INFO("Pick complete");
  state = PickState::Completed;
  return state;
}

PlaceState Arm::DoPlace() {
  logger_.INFO("Moving to prepare pose");
  PlaceState state(PlaceState::MoveToPreparePose);
  geometry_msgs::PoseStamped pose;
  if (!DoPlacePrepare(pose)) {
    logger_.WARN("Could not move to prepare pose");
    state = PlaceState::MoveToPreparePoseFailed;
    return state;
  }

  logger_.INFO("Executing approach");
  state = PlaceState::Approach;
  if (!DoPlaceApproach(pose)) {
    logger_.WARN("Could not move to place pose");
    state = PlaceState::ApproachFailed;
    return state;
  }

  logger_.INFO("Opening gripper");
  state = PlaceState::OpenGripper;
  control_msgs::GripperCommandGoal goal;
  if (!DoRelease(goal)) {
    logger_.WARN("Gripper open action did not complete");
    state = PlaceState::OpenGripperFailed;
    return state;
  }

  logger_.INFO("Retreating");
  state = PlaceState::Retreat;
  if (!DoRetreat(pose)) {
    logger_.WARN("Could not move to retreat pose");
    state = PlaceState::RetreatFailed;
    return state;
  }

  DoRest(goal);

  logger_.INFO("Place complete");
  state = PlaceState::Completed;
  return state;
}

void Arm::DoMoveVertical() {
  arm_.setNamedTarget("vertical");
  arm_.move();
}

void Arm::SetSupportSurfaceName(const std::string& surfaceName) {
  arm_.setSupportSurfaceName("table");
}

bool Arm::DoPickPrepare(geometry_msgs::PoseStamped& pose, double x, double y) {
  pose.header.frame_id = params_.scene_task_frame_;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = params_.pick_prepare_z_;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.707106;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 0.707106;
  // Plan a move to the pose
  arm_.setPoseTarget(pose);

  // Execute the move
  if (!arm_.move()) {
    return false;
  }
  return true;
}

bool Arm::DoOpenGripper(control_msgs::GripperCommandGoal& goal) {
  // Open the gripper to the configured open width
  goal.command.position = params_.gripper_open_;
  // Send the gripper command
  gripper_.sendGoal(goal);
  // Wait for the command to complete
  return gripper_.waitForResult(ros::Duration(30));
}

bool Arm::DoApproach(geometry_msgs::PoseStamped& pose) {
  // Move to the configured height above the surface to get the gripper
  // around the object
  pose.pose.position.z = params_.pick_z_;
  arm_.setPoseTarget(pose);
  if (!arm_.move()) {
    return false;
  }
  return true;
}

bool Arm::DoGrasp(control_msgs::GripperCommandGoal& goal) {
  // Close the gripper to the configured closed width
  goal.command.position = params_.gripper_close_;
  gripper_.sendGoal(goal);
  if (!gripper_.waitForResult(ros::Duration(30))) {
    return false;
  }

  //[debug]
  //memo: before reaching here, error occures.
  logger_.INFO("77777-(before attachObject in arm.cpp)");
  arm_.attachObject("pick-target", "", gripper_group_.getLinkNames());
  logger_.INFO("77777-(after attachObject in arm.cpp)");

  picking_target_attached_ = true;
  return true;
}

bool Arm::DoPlacePrepare(geometry_msgs::PoseStamped &pose) {
  pose.header.frame_id = params_.scene_task_frame_;
  pose.pose.position.x = params_.place_x_;
  pose.pose.position.y = params_.place_y_;
  pose.pose.position.z = params_.place_prepare_z_;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.707106;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 0.707106;
  arm_.setPoseTarget(pose);
  if (!arm_.move()) {
    return false;
  }
  return true;
}

bool Arm::DoPlaceApproach(geometry_msgs::PoseStamped &pose) {
  pose.pose.position.z = params_.place_z_;
  arm_.setPoseTarget(pose);
  if (!arm_.move()) {
    return false;
  }
  return true;
}

bool Arm::DoRelease(control_msgs::GripperCommandGoal& goal) {
  goal.command.position = params_.gripper_open_;
  gripper_.sendGoal(goal);
  bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
  if (!finishedBeforeTimeout) {
    return false;
  }
  arm_.detachObject("pick-target");
  picking_target_attached_ = false;
  return true;
}

bool Arm::DoRetreat(geometry_msgs::PoseStamped& pose) {
  pose.pose.position.z = params_.pick_prepare_z_;
  arm_.setPoseTarget(pose);
  if (!arm_.move()) {
    return false;
  }
  return true;
}

void Arm::DoRest(control_msgs::GripperCommandGoal &goal) {
  DoCloseGripper(goal);
  DoMoveVertical();
}

void Arm::DoCloseGripper(control_msgs::GripperCommandGoal &goal) {
  goal.command.position = params_.gripper_close_;
  gripper_.sendGoal(goal);
}

Arm::~Arm() {}
