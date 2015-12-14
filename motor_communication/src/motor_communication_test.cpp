#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"
#include <sstream>
#include <ros/console.h>
#include <sstream> 

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
typedef int16_t speed_MMpS_t;


class motor_communication
{
  private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    std::string serial_port_="/dev/ttySAC2";
    int baud_rate_=115200;
    std::string result;
    size_t bytes_wrote;
    
  public:
    void run(int speed);
    void my_sleep(unsigned long milliseconds);
    void stop();
    motor_communication(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
		  result="";
		  priv_nh_.param<std::string>("serial_port", serial_port_, "/dev/ttySAC2");
		  priv_nh_.param("baud_rate", baud_rate_,115200);
      
    }

    //! Empty stub
    ~motor_communication() {}

};
void motor_communication::my_sleep(unsigned long milliseconds) {
    usleep(milliseconds*1000); // 100 ms
}

void motor_communication::run(int speed)
{
  	try
    {
      serial::Serial my_serial(serial_port_, baud_rate_, serial::Timeout::simpleTimeout(1000));
      cout << "Is the serial port open?";
	  if(my_serial.isOpen())
	    cout << " Yes." << endl;
	  else
	    cout << " No." << endl;
	  uint8_t checksum;
	  checksum=0;
	  string test_string="en\r\n";
	  bytes_wrote =my_serial.write(test_string);
	  result = my_serial.read(test_string.length()+2);
	  ROS_INFO("read s:%s \n",result.c_str());

	  std::string speed_string = std::to_string(speed);;
	  test_string="v"+ speed_string +"\r\n";
	  bytes_wrote =my_serial.write(test_string);
	  result = my_serial.read(test_string.length()+2);
	  ROS_INFO("read speed:%s \n",result.c_str());
    }
    catch(const std::exception& e)
    {	 
      	ROS_ERROR("could not find serial port");
    }
}
void motor_communication::stop()
{
	try
	{
		serial::Serial my_serial(serial_port_, baud_rate_, serial::Timeout::simpleTimeout(1000));
		bytes_wrote =my_serial.write("di\r\n");
		result = my_serial.read(4);
		ROS_INFO("read di:%s \n",result.c_str());
	}
	catch(const std::exception& e)
	{
		ROS_ERROR("could not find serial port");
	}
}
int main(int argc, char **argv) 
{
  ros::init(argc, argv, "motor_communication_node");
  ros::NodeHandle nh;
  ros::Rate loop(100);
  motor_communication node(nh);
  
  while(ros::ok())
  {
  	node.run(1000);
  	node.stop();
    ros::spinOnce();  
    loop.sleep();
  }
  return 0;
}
