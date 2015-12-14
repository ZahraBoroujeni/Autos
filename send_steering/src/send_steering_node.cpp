#include <send_steering.h>

  send_steering servo_node;
int main(int argc, char **argv) 
{
  ros::init(argc, argv, "send_steering_node");
  ros::NodeHandle nh;
  ros::Rate loop(100);
  
  ros::spinOnce();  
  servo_node.init();
  while(ros::ok())
  {
    servo_node.run(150);
    ros::spinOnce();  
    loop.sleep();
  }
  servo_node.stop();
  return 0;
}