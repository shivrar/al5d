#include "arm_hw_integration/hw_action_server.h"

namespace arm_hw_integration{

  AL5DArm::AL5DArm(std::string name):private_nh_("~"),
    as_(private_nh_, name, boost::bind(&AL5DArm::executeCB, this, _1), false),
    action_name_(name)
  {

  };

  AL5DArm::~AL5DArm()
  {

  };

  void AL5DArm::InitialiseSubscribers()
  {

  };

  void AL5DArm::InitialiseSerial()
  {

    private_nh_.param<std::string>("port", port_, "/dev/ttyUSB0");
    private_nh_.param("baud", baud_, 9600);

    int timeout;
    private_nh_.param("timeout", timeout, 1000);
    timeout_ = serial::Timeout::simpleTimeout(timeout);

    try
    {
      serial_.reset(new serial::Serial (port_, baud_, timeout_));

      if (serial_->isOpen())
      {
        ROS_INFO("Successful connection");

        /*As noted in the ssc-32u manual:
          "The first positioning command should be a normal "# <ch> P <pw>" command.
           Because the controller doesn't know where the servo is positioned on power­up, it will ignore
           speed and time commands until the first normal command has been received."
           So initial command is to first send the first normal command and then center all the servos for the IK stuff
        */
      }
      else
      {
        ROS_WARN("FAILED connection");
      }
    }

    catch (serial::IOException& e)
    {
      ROS_ERROR("serial exception: %s, thrown, serial port does not exist", e.what());
      ros::shutdown();
    }

  };

  void AL5DArm::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {

  };
}