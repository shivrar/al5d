#include <termios.h>
#include <unistd.h>
#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <cstdio>
#include "serial/serial.h"

//TODO simple test using the termios serial or serial package interface to get basic motion pragmatically

void enumerate_ports()
{
	std::vector<serial::PortInfo> devices_found = serial::list_ports();

	std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while( iter != devices_found.end() )
	{
		serial::PortInfo device = *iter++;

		printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
     device.hardware_id.c_str() );
	}
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_hw_integration_node");

    ros::NodeHandle nh;

    enumerate_ports();

    std::string port = "/dev/ttyUSB0";

    unsigned long baud = 9600;

    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);

    try
    {
      serial::Serial test_serial(port, baud, timeout);

      if (test_serial.isOpen())
      {
        ROS_INFO("Successful connection");

        std::string command =  "​​0 P1500 T2000";
        std::string command2 = "​​#1 P1500 T2000";
        std::string command3 = "​​#2 P1500 T2000";
        std::string command4 = "​​# P1500 T2000";
        std::string command5 = "​​#0 P1500 T2000";
        std::string command6 = "​​#0 P1500 T2000";


        test_serial.write(command + "\r");

        test_serial.write("Q\r");

//        usleep(500);

        std::string result = test_serial.read(command.length()+1);

        ROS_INFO("Command sent %s, Result received %s", command.c_str(), result.c_str());
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


    ros::spin();

    return 0;
}
