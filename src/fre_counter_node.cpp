#include "ros/ros.h"
#include <std_msgs/Int64.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <nav_msgs/Path.h>

//reads the actual mode and change and actuate everything as neccessary for task 1 
//optimized for task 1 fieldrobot event 2016
//author: David Reiser
//date: counter for gazebo simulator to extract the actual team counts for 
// plants hits
// robot distance traveled
// using start position of robot to estimate the rows traveled and actual odometry since last row change...
//publish robot path and results as topics


class Fre_Counter
{
		
private:

	bool got_first_model_stages, headland_navigation;
	geometry_msgs::Pose last_robot_pose, robot_start_pose;
	float dist_robot_travel, actual_row;
	int row_counter;
	nav_msgs::Path path;
	
	
	


public:	
	gazebo_msgs::ModelStates start_model_stages, actual_model_stages;
	ros::Publisher path_pub, plants_destroyed_pub, robot_dist_pub, actual_row_dist_pub,rows_pub;

	Fre_Counter()
	{
		got_first_model_stages=false;
		dist_robot_travel=0;
		actual_row=0;
		row_counter=0;
		headland_navigation=false;
		path.header.frame_id="odom";
		path.header.stamp=ros::Time::now();
		
		
	}
	
	~Fre_Counter()
	{
	}
	
	//check if actual orientation is switched at the headland returns  rel rotation in degree
	double get_rel_rotation(geometry_msgs::Pose last, geometry_msgs::Pose actual)
	{
		//check relative rotation between the quaternions
		tf2::Quaternion a,b,c;
		tf2::convert(last.orientation , a);
		tf2::convert(actual.orientation , b);
	
		c = b * a.inverse(); //calculate relative rotation
		//convert yaw to degrees 
		tf2::Matrix3x3 m(c);
		
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		yaw=yaw*(180.0/M_PI); //convert to degree;
		
		return yaw;
	}
	

	//------subscribers----------
	//subscribe to gazebo/model_states
	void readModelStates(const gazebo_msgs::ModelStates::ConstPtr& msg)
	{
		//check how many objects are found 
		
		//std::cout<<"got state of: "<<msg->name.size() <<" objects in gazebo"<<std::endl;
		try
		{
			int plant_counter=0;
			int moved_plants=0;
			for(int i=0;i<msg->name.size();i++)
			{
				//check for plants
				if (msg->name[i].find("maize") != std::string::npos) {
					//std::cout << "found maize!" << '\n';
					plant_counter++;
					if(got_first_model_stages) //check if we can compare something until now
					{
						//check if plant changed position since first stage
						float plant_dist=sqrt((msg->pose[i].position.x-start_model_stages.pose[i].position.x)*(msg->pose[i].position.x-start_model_stages.pose[i].position.x)+(msg->pose[i].position.y-start_model_stages.pose[i].position.y)*(msg->pose[i].position.y-start_model_stages.pose[i].position.y));
						
						
						if(plant_dist>0.01)//if plant was moved more than x cm //maybe include here later the orientation of the stem?
						{
							//std::cout<<"the plant nr. "<<i<<" was moved!"<<std::endl;
							moved_plants++;
						}
					}
					
				}
				
				if(msg->name[i] == "jackal")
				{	
					
					geometry_msgs::PoseStamped point;
					point.header.stamp=ros::Time::now();
					point.header.frame_id="odom"; //push path to odom frame
					point.pose=msg->pose[i];
					path.poses.push_back(point); //push back actual position to pose
								
					if(got_first_model_stages) //check if we can compare something until now
					{
						//we can start to compare the distance 
						float dist_travel=sqrt((msg->pose[i].position.x-last_robot_pose.position.x)*(msg->pose[i].position.x-last_robot_pose.position.x)+(msg->pose[i].position.y-last_robot_pose.position.y)*(msg->pose[i].position.y-last_robot_pose.position.y));
						if (dist_travel>0.1)//add dist to robot_distance traveled and reset last_robot_pose;
						{
							if(!headland_navigation){ //add distance to traveled distance if the robot is not on headland navigation
								dist_robot_travel=round(100.0*(dist_robot_travel+dist_travel))/100.0; //round distance to cm.
								actual_row=round(100.0*(actual_row+dist_travel))/100.0; //calculate for this row
							}
							last_robot_pose=msg->pose[i]; //write back last robot pose for next iteration
						}
					}else
					{
						//save robot start position if this is the first message recieved
						last_robot_pose=msg->pose[i];
						robot_start_pose=msg->pose[i];
					}
					//check if we detected a headland turn.
					double rel_rotation=get_rel_rotation(last_robot_pose,robot_start_pose);
					rel_rotation=sqrt(rel_rotation*rel_rotation);//get abs value
					std::cout<<"robot pose x:"<<msg->pose[i].position.x <<" y:"<<msg->pose[i].position.y<<"  rel rot: "<<rel_rotation<<" [°]"<<std::endl;
					if(rel_rotation>50 &&rel_rotation<150) //check if we are right now on the headland turn:
					{
						std::cout<<"on headland"<<std::endl;
						//update row counter plus one 
						headland_navigation=true;
						//take care that the dist. is not counted until the robot has finished the turning.
					}
					if(rel_rotation>=150)
					{
						//move up row counter and reset robot start_pose;
						row_counter++;
						headland_navigation=false;
						robot_start_pose=msg->pose[i];//override robot start pose with actual pose of robot
						actual_row=0; //reset distance in actual row
					}
					
					std::cout<<"robot distance total: "<< dist_robot_travel<<" [m]"<<std::endl;
					std::cout<<"rows finished: " << row_counter<<"  distance in actual row: "<<actual_row<<" [m]"<<std::endl;
					//publish values as topics...
					std_msgs::Float64 dist;
					dist.data=dist_robot_travel;
					robot_dist_pub.publish(dist);
					dist.data=actual_row;
					actual_row_dist_pub.publish(dist);
					std_msgs::Int64 rows;
					rows.data=row_counter;
					rows_pub.publish(rows);
				}
			}
			std::cout<<"plants destroyed: "<< moved_plants <<std::endl<<std::endl;
			std_msgs::Int64 plants;
			plants.data=moved_plants;
			plants_destroyed_pub.publish(plants);
			
			//publish path..
			path_pub.publish(path);
			
		}
		catch(...)
		{
		ROS_INFO("Couldn't find 'jackal' in names of topic /gazebo/ModelStates");
		}
		
		//check if this is the first topic recieved. if yes, save it globaly for future comparison.
		if (!got_first_model_stages)
		{
				got_first_model_stages=true;
				start_model_stages=*msg;		
		}		
		
					
	}
	
	



};
//-------------------------------------------------------------------------


int main (int argc, char** argv)
{
	
	ros::init(argc,argv,"fre_counter");
	ros::NodeHandle n("~");
			
	Fre_Counter s;
	//in seconds...
	
	//mode subscriber
	ros::Subscriber g_models=n.subscribe("/gazebo/model_states", 10, &Fre_Counter::readModelStates,&s);
	//ros::Subscriber m=n.subscribe(mode_sub_str,10,&FreCounter::ActualMode,&s);
	//publishers:
	s.path_pub=n.advertise<nav_msgs::Path>("/fre_counter/real_robot_path",1);
	s.plants_destroyed_pub=n.advertise<std_msgs::Int64>("/fre_counter/plants_destroyed",1);
	s.rows_pub=n.advertise<std_msgs::Int64>("/fre_counter/rows_finished",1);
	s.robot_dist_pub=n.advertise<std_msgs::Float64>("/fre_counter/robot_dist",1);
	s.actual_row_dist_pub=n.advertise<std_msgs::Float64>("/fre_counter/robot_dist_row",1);
	
	
	ros::spin();
		
}
