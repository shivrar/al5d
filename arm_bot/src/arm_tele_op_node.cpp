#include "arm_bot/arm_tele_op.h"

//Optional node for filtering all the jointState msgs and publishing only those for the wheels

int main(int argc, char** argv)
{

  ros::init(argc, argv, "arm_tele_op");

  ros::NodeHandle n;

  // Wait until we have valid timestamps before continuing to initialize this node.
  //Needed when "/use_sim_time" is true

  arm_bot::TeleOp teleop;

  ros::spin();

  return 0;
}