#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
  position: 
      x: 3.13256366157
      y: -0.00339897022722
      z: -0.0278423536449
    orientation: 
      x: -1.85993044588e-06
      y: 7.94405933841e-06
      z: 0.00164681398813
      w: 0.999998643968

 */
void chatterCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  
  geometry_msgs::Pose start_pose;
    start_pose.position.x = msg->pose[1].position.x;
    start_pose.position.y = msg->pose[1].position.y;
    start_pose.position.z = msg->pose[1].position.z;
    start_pose.orientation.x = msg->pose[1].orientation.x;
    start_pose.orientation.y = msg->pose[1].orientation.y;
    start_pose.orientation.z = msg->pose[1].orientation.z;
    start_pose.orientation.w = msg->pose[1].orientation.w;

  ROS_INFO("I heard: [%f][%f][%f]",start_pose.position.x,start_pose.position.y,start_pose.position.z);
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/gazebo/model_states", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
}
  