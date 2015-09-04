#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

ros::Publisher pub;
ros::Subscriber sub;

float  gas,brake,wheel;

void myCallback (const sensor_msgs::Joy::ConstPtr& msg)
{

  // for (unsigned i = 0; i < msg->axes.size(); ++i) {
  //   ROS_INFO("Axis %d is now at position %f", i, msg->axes[i]);
  // }
   gas=msg->axes[0];
   pub.publish(gas);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker1");
  ros::NodeHandle n;
  pub = n.advertise<float>("/drc_vehicle_xp900/gas_pedal/cmd",1000);
  sub = n.subscribe("joy", 1000, myCallback);
  ROS_INFO("Initialization complete, now spinning.");
  ros::spin();
  return 0;
}
