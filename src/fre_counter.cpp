#include "evaluation_nodes/fre_counter.h"

// Reads the actual mode and change and actuate everything as neccessary for task 1
// Optimized for task 1 fieldrobot event 2016
// author: David Reiser
// date: counter for gazebo simulator to extract the actual team counts for
//  plants hits
//  robot distance traveled
//  using start position of robot to estimate the rows traveled and actual odometry since last row change...
// publish robot path and results as topics

namespace fre_counter
{
    FRE_Counter::FRE_Counter(ros::NodeHandle *nodehandle) : nh_(*nodehandle)
    {
        got_first_model_stages = false;
        dist_robot_travel = 0;
        actual_row = 0;
        row_counter = 0;
        headland_navigation = false;
        path_msg.header.frame_id = "odom";
        path_msg.header.stamp = ros::Time::now();

        robot_name = getRobotName();
        start_time = ros::Time::now();

        // Create publishers
        real_path_publisher_ = nh_.advertise<nav_msgs::Path>("real_robot_path", 1);
        info_publisher_ = nh_.advertise<evaluation_nodes::Count>("info", 1);

        // Create subscribers
        model_state_subscriber_ = nh_.subscribe("/gazebo/model_states", 10, &FRE_Counter::readModelStates, this);
    }

    FRE_Counter::~FRE_Counter()
    {
    }

    // Check if actual orientation is switched at the headland returns rel rotation in degree
    std::tuple<double, double, double> FRE_Counter::getRelativeRotation(geometry_msgs::Pose last, geometry_msgs::Pose actual)
    {
        // Check relative rotation between the quaternions
        tf2::Quaternion a, b, c;
        tf2::convert(last.orientation, a);
        tf2::convert(actual.orientation, b);

        c = b * a.inverse(); // Calculate relative rotation
        tf2::Matrix3x3 m(c);

        double r, p, y;
        m.getRPY(r, p, y);
        // Convert to degree
        r = r * (180.0 / M_PI); 
        p = p * (180.0 / M_PI); 
        y = y * (180.0 / M_PI); 

        return {r, p, y};
    }

    //------subscribers----------
    // subscribe to gazebo/model_states
    void FRE_Counter::readModelStates(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        info_msg.header.stamp = ros::Time::now();

        bool found_robot = false;

        // Check how many objects are found

        // std::cout<<"got state of: "<<msg->name.size() <<" objects in gazebo"<<std::endl;

        int plant_counter = 0;
        int destroyed_plants = 0;

        for (int i = 0; i < msg->name.size(); i++)
        {
            // check for plants
            if (msg->name[i].find("maize") != std::string::npos)
            {
                // std::cout << "found maize!" << '\n';
                plant_counter++;
                if (got_first_model_stages) // Check if we can compare something until now
                {
                    // If plant rotated more than 45.1 degrees, the plants should flip over and are destroyed.
                    double r, p, y;
                    std::tie(r, p, y) = getRelativeRotation(start_model_stages.pose[i], msg->pose[i]);
                    if (std::abs(r) > 45.1 || std::abs(p) > 45.1)
                    {
                        destroyed_plants++;
                    }
                }
            }

            if (msg->name[i] == robot_name)
            {
                found_robot = true;

                geometry_msgs::PoseStamped point;
                point.header.stamp = ros::Time::now();
                point.header.frame_id = "odom"; // Push path to odom frame
                point.pose = msg->pose[i];

                path_msg.header.stamp = ros::Time::now();
                path_msg.poses.push_back(point); // Push back actual position to pose

                if (got_first_model_stages) // Check if we can compare something until now
                {
                    // We can start to compare the distance
                    float dist_travel = sqrt((msg->pose[i].position.x - last_robot_pose.position.x) * (msg->pose[i].position.x - last_robot_pose.position.x) + (msg->pose[i].position.y - last_robot_pose.position.y) * (msg->pose[i].position.y - last_robot_pose.position.y));
                    if (dist_travel > 0.1) // Add dist to robot_distance traveled and reset last_robot_pose;
                    {
                        if (!headland_navigation)
                        {                                                                                 // Add distance to traveled distance if the robot is not on headland navigation
                            dist_robot_travel = round(100.0 * (dist_robot_travel + dist_travel)) / 100.0; // Round distance to cm.
                            actual_row = round(100.0 * (actual_row + dist_travel)) / 100.0;               // calculate for this row
                        }
                        last_robot_pose = msg->pose[i]; // Write back last robot pose for next iteration
                    }
                }
                else
                {
                    // Dave robot start position if this is the first message recieved
                    last_robot_pose = msg->pose[i];
                    robot_start_pose = msg->pose[i];
                }

                // Check if we detected a headland turn.
                double r, p, y;
                std::tie(r, p, y) = getRelativeRotation(last_robot_pose, robot_start_pose);
                double rel_rotation = std::abs(y);
                std::cout << "robot pose x:" << msg->pose[i].position.x << " y:" << msg->pose[i].position.y << "  rel rot: " << rel_rotation << " [°]" << std::endl;

                if (rel_rotation > 70 && rel_rotation < 160) // check if we are right now on the headland turn:
                {
                    std::cout << "on headland" << std::endl;
                    // Update row counter plus one
                    headland_navigation = true;
                    // Take care that the dist. is not counted until the robot has finished the turning.
                }

                if (rel_rotation >= 160)
                {
                    // Move up row counter and reset robot start_pose;
                    row_counter++;
                    headland_navigation = false;
                    robot_start_pose = msg->pose[i]; // override robot start pose with actual pose of robot
                    actual_row = 0;                  // reset distance in actual row
                }

                std::cout << "robot distance total: " << dist_robot_travel << " [m]" << std::endl;
                std::cout << "rows finished: " << row_counter << "  distance in actual row: " << actual_row << " [m]" << std::endl;

                info_msg.rows_finished = row_counter;
                info_msg.robot_distance_in_row = actual_row;
                info_msg.robot_distance = dist_robot_travel;
            }
        }
        std::cout << "Plants destroyed: " << destroyed_plants << std::endl
                  << std::endl;

        if (!found_robot)
        {
            ROS_WARN_STREAM_THROTTLE(5, "Couldn't find '" << robot_name << "' in names of topic /gazebo/ModelStates");
        }

        info_msg.plants_destroyed = destroyed_plants;

        // Publish path and info
        info_publisher_.publish(info_msg);
        real_path_publisher_.publish(path_msg);

        // Check if this is the first topic recieved. if yes, save it globaly for future comparison.
        if (!got_first_model_stages & ros::Time::now() - start_time > ros::Duration(1.0))
        {
            got_first_model_stages = true;
            start_model_stages = *msg;

            ROS_INFO_STREAM("Sampled initial model states!");
        }
    }

    std::string FRE_Counter::getRobotName()
    {
        ros::Rate rate(10);

        while (!nh_.hasParam("/robot_description") && ros::ok())
        {
            ROS_INFO_THROTTLE(5, "Waiting for parameter /robot_description");
            rate.sleep();
        }

        urdf::Model model;
        model.initParam("/robot_description");
        std::string robot_name = static_cast<std::string>(model.getName());

        ROS_INFO_STREAM("Found robot name '" << robot_name << "' from /robot_description");

        return robot_name;
    }
}
