#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
  position: 
      x: 3.13256366157
      y: -0.00339897022722
      z: -0.0278423536449
    orientation: 
      x: -1.85993044588e-06
      y: 7.94405933841e-06
      z: 0.00164681398813
      w: 0.999998643968

 */
void chatterCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  
  	/*geometry_msgs::Pose start_pose;
	    start_pose.position.x = msg->pose[1].position.x;
	    start_pose.position.y = msg->pose[1].position.y;
	    start_pose.position.z = msg->pose[1].position.z;
	    start_pose.orientation.x = msg->pose[1].orientation.x;
	    start_pose.orientation.y = msg->pose[1].orientation.y;
	    start_pose.orientation.z = msg->pose[1].orientation.z;
	    start_pose.orientation.w = msg->pose[1].orientation.w;*/


	static tf2_ros::TransformBroadcaster br;
  static tf2_ros::TransformBroadcaster br2;
        geometry_msgs::TransformStamped transformStamped;
  
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = msg->pose[1].position.x;
        transformStamped.transform.translation.y = msg->pose[1].position.y;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation=msg->pose[1].orientation;



        br.sendTransform(transformStamped);


        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = msg->pose[1].position.x;
        transformStamped.transform.translation.y = msg->pose[1].position.y;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation=msg->pose[1].orientation;

        br2.sendTransform(transformStamped);

  //ROS_INFO("I heard: [%f][%f][%f]",start_pose.position.x,start_pose.position.y,start_pose.position.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/gazebo/model_states", 1000, chatterCallback);

  ros::spin();
}
  