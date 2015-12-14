#include <motor_communication.h>

motor_communication::motor_communication() : my_serial("/dev/ttyUSB0",115200, serial::Timeout::simpleTimeout(1000))
{
  result="";
  //priv_nh_.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB1");
  //priv_nh_.param("baud_rate", baud_rate_,115200);
  // my_serial.close();
  // my_serial.setPort(serial_port_);
  // my_serial.setBaudrate(baud_rate_);
  // my_serial.open();
  //my_serial.setTimeout(1000);
  //my_serial.Timeout.simpleTimeout(1000)
  init();
  
}

    //! Empty stub
motor_communication::~motor_communication() {}

void motor_communication::my_sleep(unsigned long milliseconds) {
    usleep(milliseconds*1000); // 100 ms
}

void motor_communication::init()
{
  	try
    {
      cout << "Is the serial port open?";
      //cout << my_serial.getBaudrate() << endl;
	  if(my_serial.isOpen())
	    cout << " Yes." << endl;
	  else
	    cout << " No." << endl;

	  bytes_wrote =my_serial.write("en\r\n");
	  result = my_serial.read(4+2);
	  ROS_INFO("start:%s \n",result.c_str());

    }
    catch(const std::exception& e)
    {	 
      	ROS_ERROR("could not find serial port");
    }
}
void motor_communication::start()
{
    try
    {
      bytes_wrote =my_serial.write("en\r\n");
      result = my_serial.read(4+2);
      ROS_INFO("start:%s \n",result.c_str());
    }
    catch(const std::exception& e)
    {  
        ROS_ERROR("could not find serial port");
    }
}
void motor_communication::run(int speed)
{
  	try
    {
      std::string speed_string = std::to_string(speed);;
	  string test_string="v"+ speed_string +"\r\n";
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
		//serial::Serial my_serial(serial_port_, baud_rate_, serial::Timeout::simpleTimeout(1000));
		bytes_wrote =my_serial.write("di\r\n");
		result = my_serial.read(4);
		ROS_INFO("read di:%s \n",result.c_str());
	}
	catch(const std::exception& e)
	{
		ROS_ERROR("could not find serial port");
	}
}
