#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/Joy.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float64 >("/drc_vehicle_xp900/gas_pedal/cmd",1000);
    ros::Rate loop_rate(1);

    int count=0;
    while (ros::ok())
    {
        std_msgs::Float64  msg;
        msg.data=1;
        chatter_pub.publish(msg);
        ROS_INFO("I heard: [%f]", msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;

    }
    return 0;
}
