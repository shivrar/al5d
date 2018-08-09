#include "arm_hw_integration/hw_action_server.h"

namespace arm_hw_integration{

  AL5DArm::AL5DArm(std::string name):private_nh_("~"),
    as_(ros::NodeHandle(), name, boost::bind(&AL5DArm::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    InitialisePublishers();
    InitialiseTimers();
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

  void AL5DArm::InitialiseTimers()
  {
    double period;
    private_nh_.param("publish_period", period, 0.025);
    pub_period_ = ros::Duration(period);

    joint_timer_ = private_nh_.createTimer(pub_period_, boost::bind(&AL5DArm::timerCB, this, _1));
  };

  void AL5DArm::timerCB(const ros::TimerEvent&)
  {
    current_states_.header.stamp = ros::Time::now();
    joint_pub_.publish(current_states_);
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

        //ros::Duration(3).sleep();

        std::string command =  "#​​0P1500 #1P1500 #2P1500 #3P1500 #4P1500 #5P1500\r";

        serial_->write(command);

        serial_->write("Q\r");

        std::string result = serial_->read(command.length()+1);

        std::vector<std::string> joint_names = {
          "al5d_joint_1",
          "al5d_joint_2",
          "al5d_joint_3",
          "al5d_joint_4",
          "al5d_gripper"};

        while(result == "+")
        {
          ros::Duration(0.0025).sleep();
          serial_->write("Q\r");
          result = serial_->read(command.length()+1);
        }

        if (result == ".")
        {
          //TODO Upon each movement Use the query pulse width to get the the actual binaries for the servos and get the joint angles upon success

          //TODO: Query the pulse width using the QP <arg> <cr> for ssc-32u board -> issues with this since the binary format in not ASCII readable, will use virtual joints for the time being

          current_states_.name = joint_names;
          current_states_.position.assign(joint_names.size(), 0.0);

          current_states_.position[0] = 0.0;
          current_states_.position[1] = M_PI/2;
          current_states_.position[2] = -M_PI/2;
          current_states_.position[3] = 0.0;
          current_states_.position[4] = 0.0;
          current_states_.header.stamp = ros::Time::now();
          joint_pub_.publish(current_states_);
        }

      }
      else
      {
        ROS_WARN("FAILED connection");
      }
    }

    catch (serial::IOException& e)
    {
      ROS_ERROR("serial exception: (%s), thrown. Serial port not connected", e.what());
      ros::shutdown();
    }

  };

  void AL5DArm::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
    std::vector<std::string> joint_names = goal->trajectory.joint_names;
    std::vector<trajectory_msgs::JointTrajectoryPoint> joint_points = goal->trajectory.points;
    std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator point_iter = joint_points.begin();

    while(point_iter != joint_points.end())
    {
      for(int i = 0; i < joint_names.size(); i++ )
      {
        ROS_INFO("Joint Name: %s, Joint Positions: %f, Joint Velocities: %f", joint_names[i].c_str(),
                  point_iter->positions[i], point_iter->velocities[i]);
        //TODO : Do the proper mapping of the joint states to the current states member variable
        switch(joint_names[i])
          case "al5d_joint_1"
          //current_states_.position[i] = point_iter->positions[i];

      }

      feedback_.header.stamp = ros::Time::now();
      feedback_.joint_names = joint_names;
      feedback_.desired = *point_iter;

      as_.publishFeedback(feedback_);

      point_iter++;
    }

    result_.error_code = 0;
    as_.setSucceeded(result_);
  };
}