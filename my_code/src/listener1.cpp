#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Joy.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"


ros::Subscriber sub;
ros::Publisher pub_brake,pub_direction,pub_gas,pub_handBrake,pub_wheel,pub_key;
std_msgs::Float64  brake,gas,handBrake,wheel;
std_msgs::Int8 direction,key;


void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  gas.data=msg->axes[0];
  brake.data=msg->axes[4];
  handBrake.data=0;
  wheel.data=msg->axes[3];
  key.data=msg->buttons[4];
  //pub_brake.publish(brake);

  if (msg->buttons[0]==1)
  	direction.data=-1;
  if (msg->buttons[1]==1)
  	direction.data=1;
  pub_direction.publish(direction);

  pub_gas.publish(gas);
  pub_handBrake.publish(handBrake);
  pub_wheel.publish(wheel);
  //pub_key.publish(key);

  //ROS_INFO("I heard: [%f]", gas);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  pub_brake = n.advertise<std_msgs::Float64 >("/fuCar/brake_pedal/cmd",1000);
  pub_direction = n.advertise<std_msgs::Int8>("/fuCar/direction/cmd",1000);
  pub_gas = n.advertise<std_msgs::Float64 >("/fuCar/gas_pedal/cmd",1000);
  pub_handBrake = n.advertise<std_msgs::Float64 >("/fuCar/hand_brake/cmd",1000);
  pub_wheel = n.advertise<std_msgs::Float64 >("/fuCar/hand_wheel/cmd",1000);
  pub_key = n.advertise<std_msgs::Int8 >("/fuCar/key/cmd",1000);

  sub = n.subscribe("joy", 1000, joyCallback);

  ros::spin();
  return 0;
}
  