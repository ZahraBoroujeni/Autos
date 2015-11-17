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

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
typedef int16_t speed_MMpS_t;
#define FLAWLESS_PROTOCOL_PACKET_CHAR_BEGINNING 0xf0
#define FLAWLESS_PROTOCOL_PACKET_CHAR_END       0x0f
#define FLAWLESS_PROTOCOL_PACKET_CHAR_ESCAPE    0x3c
#define RPC_SUB_PROTOCOL_IDENTIFIER 1U



class motor_communication
{
  private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    std::string serial_port_;
    int baud_rate_;
    uint8_t checksum;
    uint16_t outPacketLen;
    uint8_t *rpcOutPacket = new uint8_t[outPacketLen];
     void *buffer; 
     uint8_t startByte;
     uint8_t stopByte;
     uint8_t *data;
    size_t bytes_wrote;
    std::string result;
    
  public:
    void init();
    void run(int16_t speed);
    void my_sleep(unsigned long milliseconds);
    motor_communication(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
      outPacketLen = 4;
      startByte = FLAWLESS_PROTOCOL_PACKET_CHAR_BEGINNING;
      stopByte = FLAWLESS_PROTOCOL_PACKET_CHAR_END;
      bytes_wrote =0;
      result="";
      priv_nh_.param<std::string>("serial_port", serial_port_, "/dev/ttySAC2");
      priv_nh_.param("baud_rate", baud_rate_,921600);

      //ROS_INFO("serial_port_ %s", serial_port_);
      init();
    }

    //! Empty stub
    ~motor_communication() {}

};
void motor_communication::my_sleep(unsigned long milliseconds) {
    usleep(milliseconds*1000); // 100 ms
}
void motor_communication::init()
{

  

  //string test_string;
  rpcOutPacket[0] = 8;
  *((uint8_t*)&(rpcOutPacket[1])) = 0U;
  
  //rpcOutPacket[16]=100000;
  //test_string=(string*)&;
  //std::string test_string(rpcOutPacket,rpcOutPacket+sizeof(rpcOutPacket));
  
  /* send packet start */
  
}

void motor_communication::run(int16_t speed)
{
  serial::Serial my_serial(serial_port_, baud_rate_, serial::Timeout::simpleTimeout(1000));
  cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;

  buffer=&speed;
  memcpy(&(rpcOutPacket[2]), buffer, 2);
  printf("%d,%d,%d,%d \n",rpcOutPacket[0],rpcOutPacket[1],rpcOutPacket[2],rpcOutPacket[3]);
  const uint8_t *data = (const uint8_t*)rpcOutPacket;
  checksum=0;
  bytes_wrote =my_serial.write(&startByte, sizeof(startByte));
  result = my_serial.read(bytes_wrote);
  printf("%d \n",result[0]);
  //bytes_wrote =my_serial.write("a");


  const uint8_t i_byte=RPC_SUB_PROTOCOL_IDENTIFIER;
  bytes_wrote = my_serial.write(&i_byte, sizeof(i_byte));
  checksum+=i_byte;
  result = my_serial.read(bytes_wrote);
  printf("%X \n",result[0]);
  for (uint8_t i = 0U; i < outPacketLen; ++i)
  {
    bytes_wrote =my_serial.write(&data[i], sizeof(data[i]));
    result = my_serial.read(bytes_wrote);
    printf("%X \n",static_cast<unsigned int>(result[0]));
    checksum+=data[i];
  }
  uint8_t calculatedCheckSum = ~checksum  + 1U;
  bytes_wrote =my_serial.write(&calculatedCheckSum, sizeof(calculatedCheckSum));
  result = my_serial.read(bytes_wrote);
  printf("%X \n",result[0]);

  bytes_wrote = my_serial.write(&stopByte, sizeof(stopByte));
  my_sleep(1000);
  result = my_serial.read(bytes_wrote);
  printf("%X \n",result[0]);

  cout << result.length() << ", String read: " << result << endl;
  my_sleep(1000);
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "serial_node");
  ros::NodeHandle nh;
  ros::Rate loop(100);
  motor_communication node(nh);

  while(ros::ok())
  {
    node.run(0XFFFF);
    ros::spinOnce();  
    loop.sleep();
  }
  return 0;
}
