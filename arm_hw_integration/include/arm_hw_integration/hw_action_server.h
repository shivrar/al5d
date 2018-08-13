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
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <serial/serial.h>


namespace arm_hw_integration{
  class AL5DArm{

    //TODO: Do an enumeration for the joint types etc to keep the position consistent in the arrays and other referencing situations

    public:
      enum class AL5DJoints
      {
        al5d_joint_1 = 0,
        al5d_joint_2 = 1,
        al5d_joint_3 = 2,
        al5d_joint_4 = 3,
        al5d_gripper = 4
      };

      AL5DArm(std::string name);

      ~AL5DArm();

      void InitialiseSerial();

      void InitialiseSubscribers();

      void InitialisePublishers();

      void InitialiseTimers();

      void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

      void timerCB(const ros::TimerEvent&);

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
      ros::Duration pub_period_;
      serial::Timeout timeout_;
      std::unique_ptr<serial::Serial> serial_;

      sensor_msgs::JointState current_states_;
      ros::Publisher joint_pub_;
      ros::Timer joint_timer_;
      std::map<std::string, AL5DArm::AL5DJoints> joints_;


  };
};

#endif