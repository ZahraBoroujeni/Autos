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


class send_steering
{
  private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    std::string serial_port_="/dev/ttySAC2";
    int baud_rate_=115200;
    uint16_t outPacketLen;
    uint8_t *rpcOutPacket = new uint8_t[outPacketLen];
    
     uint8_t startByte;
     uint8_t stopByte;
     uint8_t *data;
    size_t bytes_wrote;
    std::string result;
    
  public:
  	void enable();
    void run(int speed);
    void my_sleep(unsigned long milliseconds);
    void stop();
    send_steering(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
		result="";
		priv_nh_.param<std::string>("serial_port", serial_port_, "/dev/ttySAC2");
		priv_nh_.param("baud_rate", baud_rate_,115200);
    }
    //! Empty stub
    ~send_steering() {}
};

void send_steering::my_sleep(unsigned long milliseconds) {
    usleep(milliseconds*1000); // 100 ms
}
void send_steering::enable()
{
  	try
    {
      serial::Serial my_serial(serial_port_, baud_rate_, serial::Timeout::simpleTimeout(1000));
	  bytes_wrote =my_serial.write("en\r");
    }
    catch(const std::exception& e)
    {	 
      	ROS_ERROR("could not find serial port");
    }
}
void send_steering::run(int speed)
{
  	try
    {
      serial::Serial my_serial(serial_port_, baud_rate_, serial::Timeout::simpleTimeout(1000));
	  std::string steering_string = std::to_string(speed);;
	  steering_string=steering_string +"\r";
	  bytes_wrote =my_serial.write(steering_string);
    }
    catch(const std::exception& e)
    {	 
      	ROS_ERROR("could not find serial port");
    }
}

void send_steering::stop()
{
	try
	{
		serial::Serial my_serial(serial_port_, baud_rate_, serial::Timeout::simpleTimeout(1000));
		bytes_wrote =my_serial.write("di\r");
	}
	catch(const std::exception& e)
	{
		ROS_ERROR("could not find serial port");
	}
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "send_steering_node");
  ros::NodeHandle nh;
  ros::Rate loop(100);
  send_steering node(nh);
  
  while(ros::ok())
  {
  	node.enable();
  	node.run(90);
  //	node.stop();
    ros::spinOnce();  
    loop.sleep();
  }
  return 0;
}
