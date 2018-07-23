#include <termios.h>
#include <unistd.h>
#include "serial/serial.h"
#include <ros/ros.h>
#include <vector>

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

    enumerate_ports();

    ros::spin();

    return 0;
}
