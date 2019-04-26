#include "arm_bot/arm_tele_op.h"

namespace arm_bot{

  TeleOp::TeleOp():private_nh_("~")
    {
      ROS_INFO("Initializing");

      initialisePublishers();
      initialiseParameters();
      initialiseSubscribers();

    };

  TeleOp::~TeleOp()
    {

    };

  void TeleOp::initialisePublishers()
    {

    };

  void TeleOp::initialiseParameters()
    {

    };

  void TeleOp::initialiseSubscribers()
    {

    };

}; //end namespace arm_bot
