#include <ros/ros.h>

#include "gripper.h"
#include "arm.h"
#include "pick_and_placer.h"
#include "get_ros_params.h"




class PickAndPlacerNode {
  PickNPlacer& pnp_;

public:
  PickAndPlacerNode(PickNPlacer& pnp) : pnp_(pnp) {}

  void DoPickAndPlace(const geometry_msgs::Pose2D::ConstPtr &msg) {
    pnp_.DoPickAndPlace(msg->x, msg->y);
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "pickandplacer");

  ros::Subscriber sub;

  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  
  // Create an instance of the class that implements the node's behaviour
  PickNPlacerParams params;
  GetROSParams(params);

  Gripper gripper("/crane_x7/gripper_controller/gripper_cmd", "true");
  Logger logger;
  Arm arm(gripper, logger, "arm", "gripper", params);
  PlanningScene scene(arm, logger);

  PickNPlacer pnp(arm, logger, scene);
  auto node = PickAndPlacerNode(pnp);

  // 1. Subscribe to the "/block" topic to receive object positions.
  // 2. Excecute DoPickAndPlace() when one is received.

  //[debug]
  logger.INFO("77777-(before DoPickAndPlace)");
  sub = nh.subscribe("/block", 1, &PickAndPlacerNode::DoPickAndPlace, &node);
  logger.INFO("77777-(after DoPickAndPlace)");

  // Wait until the node is shut down
  ros::waitForShutdown();

  ros::shutdown();
  return 0;
}
