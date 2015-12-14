#include <motor_communication.h>

motor_communication speed;
void manualControlCallback(const std_msgs::Int16 speed_value)
{
  
  int a;
  a = speed_value.data;
  speed.run(a);
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "manual_control_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe( "/manual_control/speed", 1, manualControlCallback);
   while(ros::ok())
  {
    ros::spinOnce();  
  }
  return 0;
}
