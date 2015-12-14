#include <motor_communication.h>
#include <send_steering.h>
motor_communication speed;
send_steering steering;
void manualSpeedCallback(const std_msgs::Int16 speed_value)
{
  int a;
  a = speed_value.data;
  speed.run(a);
}
void manualSteeringCallback(const std_msgs::Int16 steering_value)
{ 
  int a;
  a = steering_value.data;
  steering.run(a);
}
void manualStopStartCallback(const std_msgs::Int16 stop_value)
{ 
  if (stop_value.data==1)
  { 
    speed.stop();
    steering.stop();
  }
  else
  {
    speed.start();
    steering.start();
  }
  
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "manual_control_node");
  ros::NodeHandle nh;
  ros::Subscriber sub_speed = nh.subscribe( "/manual_control/speed", 1, manualSpeedCallback);
  ros::Subscriber sub_steering = nh.subscribe( "/manual_control/steering", 1, manualSteeringCallback);
  ros::Subscriber sub_stop = nh.subscribe( "/manual_control/stop_start", 1, manualStopStartCallback);

   while(ros::ok())
  {
    ros::spinOnce();  
  }
  return 0;
}
