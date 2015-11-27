#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
//generic C/C++ include
#include <string>
#include <sstream>


int main(int argc, char **argv)
{
    ros::init(argc,argv,"talker");
    ros::NodeHandle n;
    //ros::Publisher chatter_pub = n.advertise<std_msgs::Float64 >("/drc_vehicle_xp900/gas_pedal/cmd",1000);


    ros::Publisher chatter_pub = n.advertise<ackermann_msgs::AckermannDriveStamped >("ackermann_vehicle/ackermann_cmd",1000);

    //ros::Rate loop_rate(1);

    int count=0;
    while (ros::ok())
    {
        // std_msgs::Float64  msg;
        // msg.data=1;
        // chatter_pub.publish(msg);
        ackermann_msgs::AckermannDriveStamped msg;
        msg.header.stamp = ros::Time::now();
        //msg.drive.push_back([1,0,1,0,0]); 
        msg.drive.steering_angle=0;
        msg.drive.steering_angle_velocity=0;
        msg.drive.speed=argc;
        msg.drive.acceleration=0;
        msg.drive.jerk=0;
        chatter_pub.publish(msg);
        ROS_INFO("I heard: []");
        ros::spinOnce();
        //loop_rate.sleep();
        ++count;

    }
    return 0;
}
