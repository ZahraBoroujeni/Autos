#include <send_steering.h>

send_steering::send_steering() : servo_serial("/dev/ttyUSB1",115200, serial::Timeout::simpleTimeout(1000))
{
  result="";
  //priv_nh_.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB1");
  //priv_nh_.param("baud_rate", baud_rate_,115200);
  // servo_serial.close();
  // servo_serial.setPort(serial_port_);
  // servo_serial.setBaudrate(baud_rate_);
  // servo_serial.open();
  //my_serial.setTimeout(1000);
  //my_serial.Timeout.simpleTimeout(1000)
  init();
  
}

    //! Empty stub
send_steering::~send_steering() {}
void send_steering::init()
{
  	try
    {
      cout << "Is the servo serial port open?";
      //cout << my_serial.getBaudrate() << endl;
  	  if(servo_serial.isOpen())
  	    cout << " Yes." << endl;
  	  else
  	    cout << " No." << endl;
  	  bytes_wrote =servo_serial.write("en\r");
    }
    catch(const std::exception& e)
    {	 
      ROS_ERROR("could not find serial port");
    }
}
void send_steering::start()
{
    try
    {
      bytes_wrote =servo_serial.write("en\r");
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
      std::string speed_string = std::to_string(speed);;
	    string test_string=speed_string +"\r";
	    bytes_wrote =servo_serial.write(test_string);
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
		//serial::Serial my_serial(serial_port_, baud_rate_, serial::Timeout::simpleTimeout(1000));
		bytes_wrote =servo_serial.write("di\r");
	}
	catch(const std::exception& e)
	{
		ROS_ERROR("could not find serial port");
	}
}
