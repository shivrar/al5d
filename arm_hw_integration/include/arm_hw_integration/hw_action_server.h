#ifndef ARM_HW_INTEGRATION_AL5D_ARM_H
#define ARM_HW_INTEGRATION_AL5D_ARM_H

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <string>
#include <memory>
#include <map>
#include <ros/console.h>
#include <math.h>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <serial/serial.h>



namespace arm_hw_integration{
  class AL5DArm{

    public:

      AL5DArm(std::string name);

      ~AL5DArm();

      void InitialiseSerial();

      void InitialiseSubscribers();

      void InitialisePublishers();

      void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

      int convertJointAngleToPosition(int jointAngle);


    private:
      ros::NodeHandle private_nh_;
      actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;

      //Action server stuff
      std::string action_name_;
      control_msgs::FollowJointTrajectoryFeedback feedback_;
      control_msgs::FollowJointTrajectoryResult result_;

      //Serial stuff
      std::string port_;
      int baud_;
      serial::Timeout timeout_;
      std::unique_ptr<serial::Serial> serial_;

      sensor_msgs::JointState current_states_;
      ros::Publisher joint_pub_;
      ros::Timer joint_timer_;

  };
};

#endif