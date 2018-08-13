#include "arm_hw_integration/hw_action_server.h"

//Helper template for mapping values

template<typename tVal>
tVal map_value(std::pair<tVal,tVal> a, std::pair<tVal, tVal> b, tVal inVal)
{
  tVal inValNorm = inVal - a.first;
  tVal aUpperNorm = a.second - a.first;
  tVal normPosition = inValNorm / aUpperNorm;

  tVal bUpperNorm = b.second - b.first;
  tVal bValNorm = normPosition * bUpperNorm;
  tVal outVal = b.first + bValNorm;

  return outVal;
}

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

        joints_["al5d_joint_1"] = AL5DJoints::al5d_joint_1;
        joints_["al5d_joint_2"] = AL5DJoints::al5d_joint_2;
        joints_["al5d_joint_3"] = AL5DJoints::al5d_joint_3;
        joints_["al5d_joint_4"] = AL5DJoints::al5d_joint_4;
        joints_["al5d_gripper"] = AL5DJoints::al5d_gripper;

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
    std::map<std::string, AL5DArm::AL5DJoints>::iterator iter;
    bool success = true;


    while(point_iter != joint_points.end())
    {
      std::string command0 = "#0";
      std::string command1 = "#1";
      std::string command2 = "#2";
      std::string command3 = "#3";
      std::string command4 = "#5";
      std::string command_to_send = "";

//      if (as_.isPreemptRequested() || !ros::ok())
//      {
//        as_.setPreempted();
//        success = false;
//        break;
//      }

      for(int i = 0; i < joint_names.size(); i++ )
      {
        //ROS_INFO("Joint Name: %s, Joint Positions: %f, Joint Velocities: %f", joint_names[i].c_str(),
        //point_iter->positions[i], point_iter->velocities[i]);

        //See if the joint exists in the standard joints for the arm, if it isn't just go to the next joint in the list
        iter = joints_.find(joint_names[i]);
        if (iter == joints_.end())
        {
          //TODO: If an invalid joint is found I should remove the corresponding data for it
          //however for the time being I will just ignore it and only update the proper joints to the tf tree using the current_states_
          //This leads to some ugly branching logic so TODO: try to find a cleaner way to do this
          ROS_ERROR("Unknown joint found: %s", joint_names[i].c_str());
          continue;
        }

        switch(iter->second)
        {
          // The integers cast represents the channel number for eg. static_cast<int>(AL5DJoints::al5d_joint_1) maps to channel "0"
//          TODO: Address the issue where some speeds generated are to large and causing jerky motion, will tackle in a future PR
          case AL5DJoints::al5d_joint_1:
          {
            std::pair<double,double> a(M_PI/2,-M_PI/2), b(500,2500);
            std::string speed;

            if((point_iter->velocities[i]/point_iter->positions[i])*map_value(a,b, point_iter->positions[i]) == -0.0  ||
               (point_iter->velocities[i]/point_iter->positions[i])*map_value(a,b, point_iter->positions[i]) == 0.0)
            {
              speed = "";
            }
            else
            {
              speed = " S" +std::to_string((int)round(std::abs((point_iter->velocities[i]/point_iter->positions[i]) * map_value(a,b, point_iter->positions[i]))));
            }

            command0 = command0 +" P"+ std::to_string((int)round(map_value(a,b, point_iter->positions[i]))) + speed;
//            ROS_INFO("Command 0: %s", command0.c_str());
            current_states_.position[static_cast<int>(AL5DJoints::al5d_joint_1)] = point_iter->positions[i];
            break;
          }
          case AL5DJoints::al5d_joint_2:
          {
            std::pair<double,double> a(0,M_PI), b(500,2500);
            std::string speed;

            if((point_iter->velocities[i]/point_iter->positions[i])*map_value(a,b, point_iter->positions[i]) == -0.0  ||
               (point_iter->velocities[i]/point_iter->positions[i])*map_value(a,b, point_iter->positions[i]) == 0.0)
            {
              speed = "";
            }
            else
            {
              speed = " S" +std::to_string((int)round(std::abs((point_iter->velocities[i]/point_iter->positions[i]) * map_value(a,b, point_iter->positions[i]))));
            }

            command1 = command1 +" P"+ std::to_string((int)round(map_value(a,b, point_iter->positions[i]))) + speed;
//            ROS_INFO("Command 1: %s", command1.c_str());
            current_states_.position[static_cast<int>(AL5DJoints::al5d_joint_2)] = point_iter->positions[i];
            break;
          }
          case AL5DJoints::al5d_joint_3:
          {
            std::pair<double,double> a(0,-M_PI), b(500,2500);
            std::string speed;

            if((point_iter->velocities[i]/point_iter->positions[i])*map_value(a,b, point_iter->positions[i]) == -0.0  ||
               (point_iter->velocities[i]/point_iter->positions[i])*map_value(a,b, point_iter->positions[i]) == 0.0)
            {
              speed = "";
            }
            else
            {
              speed = " S" +std::to_string((int)round(std::abs((point_iter->velocities[i]/point_iter->positions[i]) * map_value(a,b, point_iter->positions[i]))));
            }

            command2 = command2 +" P"+ std::to_string((int)round(map_value(a,b, point_iter->positions[i]))) + speed;
//            ROS_INFO("Command 2: %s", command2.c_str());
            current_states_.position[static_cast<int>(AL5DJoints::al5d_joint_3)] = point_iter->positions[i];
            break;
          }
          case AL5DJoints::al5d_joint_4:
          {
            std::pair<double,double> a(-M_PI/2,M_PI/2), b(500,2500);
            std::string speed;

            if((point_iter->velocities[i]/point_iter->positions[i])*map_value(a,b, point_iter->positions[i]) == -0.0  ||
               (point_iter->velocities[i]/point_iter->positions[i])*map_value(a,b, point_iter->positions[i]) == 0.0)
            {
              speed = "";
            }
            else
            {
              speed = " S" +std::to_string((int)round(std::abs((point_iter->velocities[i]/point_iter->positions[i]) * map_value(a,b, point_iter->positions[i]))));
            }

            command3 = command3 +" P"+ std::to_string((int)round(map_value(a,b, point_iter->positions[i]))) + speed;
//            ROS_INFO("Command 3: %s", command3.c_str());
            current_states_.position[static_cast<int>(AL5DJoints::al5d_joint_4)] = point_iter->positions[i];
            break;
          }
          case AL5DJoints::al5d_gripper:
          {
            std::pair<double,double> a(M_PI/2,-M_PI/2), b(500,2500);
            std::string speed;

            if((point_iter->velocities[i]/point_iter->positions[i])*map_value(a,b, point_iter->positions[i]) == -0.0  ||
               (point_iter->velocities[i]/point_iter->positions[i])*map_value(a,b, point_iter->positions[i]) == 0.0)
            {
              speed = "";
            }
            else
            {
              speed = " S" +std::to_string((int)round(std::abs((point_iter->velocities[i]/point_iter->positions[i]) * map_value(a,b, point_iter->positions[i]))));
            }
//          TODO: Address the issue with the gripper being mismatched with it's channel
            command4 = command4 +" P"+ std::to_string((int)round(map_value(a,b, point_iter->positions[i]))) + speed;
//            ROS_INFO("Command 4: %s", command4.c_str());
            current_states_.position[static_cast<int>(AL5DJoints::al5d_gripper)] = point_iter->positions[i];
            break;
          }
        }


      }

      command_to_send = command0+" "+command1+" "+command2+" "+command3+" "+command4+"\r";

      ROS_INFO("Final command: %s", command_to_send.c_str());

      serial_->write(command_to_send);

//      serial_->write("Q\r");

//      std::string result = serial_->read(command_to_send.length()+1);

//      while(result == "+")
//      {
////        usleep(50);
//        serial_->write("Q\r");
//        result = serial_->read(command_to_send.length()+1);
//        ROS_INFO("Result, %s \n", result.c_str());
//      }

      feedback_.header.stamp = ros::Time::now();
      feedback_.joint_names = joint_names;
      feedback_.desired = *point_iter;

      as_.publishFeedback(feedback_);

      ros::Duration(0.00005).sleep();

      point_iter++;
    }
//TODO: See to fix up this preemption properly
    result_.error_code = 0;
    as_.setSucceeded(result_);
//    if(success)
//    {
//      result_.error_code = 0;
//      as_.setSucceeded(result_);
//    }

  };
}