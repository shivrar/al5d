#include "arm_hw_integration/hw_action_server.h"

namespace arm_hw_integration{

  AL5DArm::AL5DArm(std::string name):private_nh_("~"),
    as_(private_nh_, name, boost::bind(&AL5DArm::executeCB, this, _1), false),
    action_name_(name)
  {
    InitialisePublishers();
    InitialiseSerial();



  };

  AL5DArm::~AL5DArm()
  {

  };

  void AL5DArm::InitialiseSubscribers()
  {

  };

  void AL5DArm::InitialisePublishers()
  {
    joint_pub_ = ros::NodeHandle().advertise<sensor_msgs::JointState>("joint_states", 10);
  };

  int AL5DArm::convertJointAngleToPosition(int jointAngle)
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

        ros::Duration(3).sleep();

        std::string command =  "#​​0P1500 #1P1500 #2P1500 #3P1500 #4P1500 #5P1500\r";

        serial_->write(command);

        serial_->write("Q\r");

//        usleep(500);

        std::string result = serial_->read(command.length()+1);

        std::vector<std::string> joint_names = {
          "al5d_joint_1,",
          "al5d_joint_2",
          "al5d_joint_3",
          "al5d_joint_4",
          "al5d_gripper"};

        if(result == "+")
        {
          //TODO: CONVERT THESE TO PRIVATE  MEMBER VARIABLES AND REPUBLISH AT A RATE
          current_states_.name = joint_names;
          current_states_.position.assign(joint_names.size(), 0.0);

          current_states_.position[0] = 0.0;
          current_states_.position[1] = M_PI/2;
          current_states_.position[2] = -M_PI/2;
          current_states_.position[3] = 0.0;
          current_states_.position[4] = 0.0;
          current_states_.header.stamp = ros::Time::now();
          joint_pub_.publish(msg);
        }

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