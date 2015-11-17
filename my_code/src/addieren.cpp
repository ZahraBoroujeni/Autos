#include "ros/ros.h"
#include <iostream>
using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "addieren");
    ros::NodeHandle n;
    int numbers[1]={0};
    //int * p;
    numbers[0]+=10;

   // *p = numbers[0]+10;
      cout << numbers[0] << ", ";
    return 0;
}
