#include <ros/ros.h>

#include "gripper.h"
#include "arm.h"
#include "pick_and_placer.h"
#include "get_ros_params.h"

#include "pick_and_placer_msgs/DoPick.h"
#include "pick_and_placer_msgs/DoPlace.h"
#include "pick_and_placer_msgs/DoMoveVertical.h"
#include "pick_and_placer_msgs/Initialize.h"

typedef pick_and_placer_msgs::DoPick DoPick;
typedef pick_and_placer_msgs::DoPlace DoPlace;
typedef pick_and_placer_msgs::DoMoveVertical DoMoveVertical;
typedef pick_and_placer_msgs::Initialize Initialize;

class PickAndPlacerServices {
  PickNPlacer& pnp_;
  Logger& logger_;
  Arm& arm_;

public:
  PickAndPlacerServices(PickNPlacer& pnp, Logger& logger, Arm& arm) : pnp_(pnp), logger_(logger), arm_(arm) {}

  bool DoMoveVerticalService(DoMoveVertical::Request& req, DoMoveVertical::Response& res) {
    pnp_.DoMoveVertical();
    return 1;
  }

  bool DoPickService(DoPick::Request& req, DoPick::Response& res) {
    bool picked = pnp_.DoPick(req.pose.x, req.pose.y);
    res.success = picked;
    return 1;
  }

  bool DoPlaceService(DoPlace::Request& req, DoPlace::Response& res) {
    PickNPlacerParams params;
    GetROSParams(params);
    arm_.SetParams(params);
    bool placed = pnp_.DoPlace();
    res.success = placed;
    return 1;
  }

  bool InitializeService(Initialize::Request& req, Initialize::Response& res) {
    pnp_.Initialize();
    return 1;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pickandplacer");
  ROS_INFO("Starting pick and place service");

  ros::NodeHandle nh;
  // Create an instance of the class that implements the node's behaviour
  PickNPlacerParams params;
  GetROSParams(params);

  ROS_INFO("Initializing gripper");
  Gripper gripper("/crane_x7/gripper_controller/gripper_cmd", "true");
  ROS_INFO("Initializing logger");
  Logger logger;
  ROS_INFO("Initializing arm");
  Arm arm(gripper, logger, "arm", "gripper", params);
  ROS_INFO("Initializing planning scene");
  PlanningScene scene(arm, logger);

  ROS_INFO("Initializing pick and placer");
  PickNPlacer pnp(arm, logger, scene);
  ROS_INFO("Initializing services");
  auto services = PickAndPlacerServices(pnp, logger, arm);

  ROS_INFO("Initializing pick service");
  auto pickService = nh.advertiseService("do_pick", &PickAndPlacerServices::DoPickService, &services);
  ROS_INFO("Initializing place service");
  auto placeService = nh.advertiseService("do_place", &PickAndPlacerServices::DoPlaceService, &services);
  ROS_INFO("Initializing move vertical service");
  auto doMoveVerticalService = nh.advertiseService("do_move_vertical", &PickAndPlacerServices::DoMoveVerticalService, &services);
  ROS_INFO("Initializing initialize service");
  auto initializeService = nh.advertiseService("initialize", &PickAndPlacerServices::InitializeService, &services);

  ROS_INFO("Pick and place service started");

  // Wait until the node is shut down
  ros::waitForShutdown();

  ros::shutdown();
  return 0;
}
