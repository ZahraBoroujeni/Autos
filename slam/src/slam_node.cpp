#include <fstream>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>


#include <Eigen/Eigen>
namespace transformations {

class online_tf
{
  private:
    // the node handle
    ros::NodeHandle nh_;

    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    // subscribers
    ros::Subscriber read_map_positions;

    tf::TransformBroadcaster tf_broadcaster_;

    // publishers

    std::string start_frame;
    std::string end_frame;
    std::vector<int> marker_id;

    Eigen::MatrixXd map_points;
    Eigen::MatrixXd read_points;

    Eigen::VectorXd transfParameters;


  public:
    std::string map_file_path;

    ros::Publisher pub_transform_;
    visualization_msgs::MarkerArray feature_markers_;

    // callback functions
    void read_map_coordinates(std::string,int);
    // constructor
    online_tf(ros::NodeHandle nh, int argc,char** argv) : nh_(nh), priv_nh_("~")
    {
        priv_nh_.param<std::string>("map_file", map_file_path, "");
        priv_nh_.param<std::string>("start_frame", start_frame, "");
        priv_nh_.param<std::string>("end_frame", end_frame, "");


        int max_id=-1;
        marker_id.resize(argc-1);
        for (int i=0;i<argc-1;i++)
        {	marker_id[i]=std::atoi(argv[i+1]);
            if (marker_id[i]>max_id)
                max_id=marker_id[i];
        }

        transfParameters.resize(7);
        pub_transform_= nh.advertise<visualization_msgs::MarkerArray>(nh.resolveName("/Features_markers"), 1);
        read_map_coordinates(map_file_path,16);


    }

    void inizialize_markers(visualization_msgs::Marker&);
    //! Empty stub
    ~online_tf() {}

};

void online_tf::inizialize_markers(visualization_msgs::Marker& marker)
{
    marker.header.frame_id = "/world";
    marker.ns = "/Features";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
}

void online_tf::read_map_coordinates(std::string map_file_path,int max_id)
{
    std::ifstream f_in(map_file_path.c_str());
    int id;
    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker mark;


    map_points=Eigen::MatrixXd::Zero(max_id+1,4);

    if (!f_in)
        ROS_ERROR("Error in opening file %s",map_file_path.c_str());
     //ROS_ERROR("Error in reading file %s",f_in>>);
    while(f_in>>id)
    {	

        if (!(f_in>>map_points(id,0)>>map_points(id,1)>>map_points(id,2)))
            ROS_ERROR("Error in reading file %s",map_file_path.c_str());

        std::cout<<id<<" "<<map_points(id,0)<<" "<<map_points(id,1)<<" "<<map_points(id,2)<<std::endl;
        inizialize_markers(mark);  
        mark.pose.position.x = map_points(id,0);
        mark.pose.position.y = map_points(id,1);
        mark.pose.position.z = map_points(id,2);
        mark.id = id;
        if (int(id/4)==0)
            mark.color.r = 1;
        if (int(id/4)==1)
            mark.color.g = 1;
        if (int(id/4)==2)
            mark.color.b = 1;
        if (int(id/4)==3)
            mark.color.a = 1;
        feature_markers_.markers.push_back(mark);
    
    }
    pub_transform_.publish(feature_markers_);
}

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "online_tf");

    //ROS_ERROR("Nargs %d val=%s e % s", argc,argv[0],argv[1]);

    ros::NodeHandle nh;
    transformations::online_tf node(nh,argc,argv);


    while(ros::ok())
    {
        node.pub_transform_.publish(node.feature_markers_);
        ros::spinOnce();

    }

    return 0;
}
