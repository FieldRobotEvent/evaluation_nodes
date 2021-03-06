#ifndef FRE_COUNTER_H
#define FRE_COUNTER_H

#include <tuple>

#include <ros/ros.h>
#include <urdf/model.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <evaluation_nodes/Count.h>

// reads the actual mode and change and actuate everything as neccessary for task 1
// optimized for task 1 fieldrobot event 2016
// author: David Reiser
// date: counter for gazebo simulator to extract the actual team counts for
//  plants hits
//  robot distance traveled
//  using start position of robot to estimate the rows traveled and actual odometry since last row change...
// publish robot path and results as topics

namespace fre_counter
{

	class FRE_Counter
	{

	public:
		FRE_Counter(ros::NodeHandle *nodehandle);
		~FRE_Counter();

		std::string getRobotName();
		std::tuple<double, double, double> getRelativeRotation(geometry_msgs::Pose last, geometry_msgs::Pose actual);
		void readModelStates(const gazebo_msgs::ModelStates::ConstPtr &msg);

	private:
		ros::NodeHandle nh_;

		ros::Subscriber model_state_subscriber_;
		ros::Publisher real_path_publisher_, info_publisher_;

		gazebo_msgs::ModelStates start_model_stages;
		geometry_msgs::Pose last_robot_pose, robot_start_pose;
		nav_msgs::Path path_msg;
		evaluation_nodes::Count info_msg;

		bool got_first_model_stages, headland_navigation;
		float dist_robot_travel, actual_row;
		int row_counter;
		std::string robot_name;
		ros::Time start_time;
	};

}
#endif // FRE_COUNTER_H