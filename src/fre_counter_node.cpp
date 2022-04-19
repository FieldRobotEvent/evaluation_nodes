#include <ros/ros.h>

#include "evaluation_nodes/fre_counter.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "fre_counter");
	ros::NodeHandle n("~");

	fre_counter::FRE_Counter counter(n);
	
	ros::spin();
}
