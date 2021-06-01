#include "ros/ros.h"
#include <std_msgs/Int64.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>

//reads the actual mode and change and actuate everything as neccessary for task 1 
//optimized for task 1 fieldrobot event 2016
//author: David Reiser
//date: counter for gazebo simulator to extract the actual team counts for 
// plants hits
// robot distance traveled
// using start position of robot to estimate the rows traveled and actual odometry since last row change...




class Fre_Counter
{
		
private:

	bool got_first_model_stages;
	geometry_msgs::Pose last_robot_pose;
	float dist_robot_travel;
	


public:	
	gazebo_msgs::ModelStates start_model_stages, actual_model_stages;
	

	Fre_Counter()
	{
		got_first_model_stages=false;
		dist_robot_travel=0;
		
		
	}
	
	~Fre_Counter()
	{
	}
	

	//------subscribers----------
	//subscribe to gazebo/model_states
	void readModelStates(const gazebo_msgs::ModelStates::ConstPtr& msg)
	{
		//check how many objects are found 
		std::cout<<"got state of: "<<msg->name.size() <<" objects in gazebo"<<std::endl;
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
						
						
						if(plant_dist>0.01)//if plant was moved more than one cm //maybe include here later the orientation of the stem?
						{
							//std::cout<<"the plant nr. "<<i<<" was moved!"<<std::endl;
							moved_plants++;
						}
					}
					
				}
				
				if(msg->name[i] == "jackal")
				{				
					if(got_first_model_stages) //check if we can compare something until now
					{
						//we can start to compare the distance 
						float dist_travel=sqrt((msg->pose[i].position.x-last_robot_pose.position.x)*(msg->pose[i].position.x-last_robot_pose.position.x)+(msg->pose[i].position.y-last_robot_pose.position.y)*(msg->pose[i].position.y-last_robot_pose.position.y));
						if (dist_travel>0.1)//asdd dist to robot_distance traveled and reset last_robot_pose;
						{
							dist_robot_travel=round(10.0*(dist_robot_travel+dist_travel))/10.0;
							
							last_robot_pose=msg->pose[i];
						}
					}else
					{
						//save robot start position:
						last_robot_pose=msg->pose[i];
					}
					//std::cout<<" we found the robot at position:"<<std::endl;
					//std::cout<<msg->pose[i]<<std::endl;
					std::cout<<"the robot traveled "<< dist_robot_travel<<" meters until now"<<std::endl;
				}
			}
			std::cout<<"found "<<plant_counter<<" plants at the field "<< moved_plants <<" were destroyed by the robot"<<std::endl<<std::endl;;
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
	
	
	ros::spin();
		
}
