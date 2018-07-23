#ifndef ARM_BOT_ARM_TELE_OP_H
#define ARM_BOT_ARM_TELE_OP_H

#include <ros/ros.h>
#include <string>
#include <memory>
#include <map>
#include <ros/console.h>
#include <math.h>
#include <vector>

//Header files related to message types used
#include <sensor_msgs/JointState.h>

//For transform support
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


namespace arm_bot{
  /**
   * @class TeleOp
   * @brief TODO Brief for what the class does.
   */
  class TeleOp{
    public:

      TeleOp();

      ~TeleOp();

      void initialiseSubscribers();

      void initialisePublishers();

      void initialiseParameters();

    private:
      ros::NodeHandle private_nh_;
      ros::Subscriber joint_sub_;
      tf::TransformListener listener_;
  };
};  // namespace arm_bot

#endif
