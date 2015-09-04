#include <fstream>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include "kalman.h"

#include "tf_frames/Transform.h"
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

    ros::Publisher pub_transform_;

    std::string map_file_path;
    std::string start_frame;
    std::string end_frame;
    std::vector<int> marker_id;

    Eigen::MatrixXd map_points;
    Eigen::MatrixXd read_points;

    kalman filter;

    Eigen::VectorXd transfParameters;


    void read_map_coordinates(std::string,int);


  public:
//

    // callback functions
    void calculate_tf(const visualization_msgs::MarkerArray & msg);

   void OptimalRigidTransformation(Eigen::MatrixXd startP, Eigen::MatrixXd finalP);


    // constructor
    online_tf(ros::NodeHandle nh, int argc,char** argv) : nh_(nh), priv_nh_("~"), filter()
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

        read_map_coordinates(map_file_path,max_id);
        transfParameters.resize(7);
                // subscribe to topics
        read_map_positions = nh_.subscribe(nh_.resolveName("/circle_markers"),
                                                  10, &online_tf::calculate_tf,this);
        pub_transform_= nh.advertise<tf_frames::Transform>(nh.resolveName("transform"), 10);
    }

    //! Empty stub
    ~online_tf() {}

};



void online_tf::read_map_coordinates(std::string map_file_path,int max_id)
{
    std::ifstream f_in(map_file_path.c_str());
    int id;

    map_points=Eigen::MatrixXd::Zero(max_id+1,4);

    if (!f_in)
        ROS_ERROR("Error in opening file %s",map_file_path.c_str());
    while(f_in>>id)
    {	//std::cout<<"id1 "<<id<<std::endl;
        if (!(f_in>>map_points(id,0)>>map_points(id,1)>>map_points(id,2)))
            ROS_ERROR("Error in reading file %s",map_file_path.c_str());

    }
}


// this function is called when a new message is received at the topic_name_you_want_to_subscribe_to
void online_tf::calculate_tf(const visualization_msgs::MarkerArray & msg)
{	tf::Transform tr;
    read_points=Eigen::MatrixXd::Zero(msg.markers.size(),4);
    int numrows=0;

    for (int i=0;i<msg.markers.size();i++)
    {	//check if the markers id is in the list and if it has not be read yet
        if((find(marker_id.begin(), marker_id.end(), msg.markers[i].id) != marker_id.end())) // && read_points((msg.markers[i].id,0)==0))
        //if (read_points((msg.markers[i].id,0)==0))
        {	read_points(numrows,0)=msg.markers[i].id;
            read_points(numrows,1)=msg.markers[i].pose.position.x;
            read_points(numrows,2)=msg.markers[i].pose.position.y;
            read_points(numrows,3)=msg.markers[i].pose.position.z;
            numrows++;
        }

    }


    Eigen::MatrixXd appo=Eigen::MatrixXd::Zero(numrows,4);
    for (int i=0;i<numrows;i++)
        appo.row(i)=cad_points.row(read_points(i,0));



    Eigen::MatrixXd startP=appo.block(0,0,numrows,3);

    Eigen::MatrixXd finalP=read_points.block(0,1,numrows,3);

    if (filter.getFirst()==0)
    {
        filter.setFirst(1);
        OptimalRigidTransformation(startP,finalP);
        filter.setKalman_x(transfParameters);


    }
    else
    {
        filter.prediction(startP);
        transfParameters=filter.update(finalP);
    }

    tr.setOrigin( tf::Vector3(transfParameters(0),transfParameters(1),transfParameters(2)));
    tr.setRotation( tf::Quaternion(transfParameters(3),transfParameters(4),transfParameters(5),transfParameters(6)));

    tf_frames::Transform msg_t;

    msg_t.header.stamp = ros::Time::now();

    tf::transformTFToMsg(tr,msg_t.Transf);
    pub_transform_.publish(msg_t);

    tf_broadcaster_.sendTransform(tf::StampedTransform(tr, ros::Time::now(), start_frame.c_str(), end_frame.c_str()));

}

void online_tf::OptimalRigidTransformation(Eigen::MatrixXd startP, Eigen::MatrixXd finalP)
{
    Eigen::Matrix4d transf;

    if (startP.rows()!=finalP.rows())
    {	ROS_ERROR("The number of rows of startP and finalP have to be the same");
        exit(1);
    }

    Eigen::RowVector3d centroid_startP=Eigen::RowVector3d::Zero();
    Eigen::RowVector3d centroid_finalP=Eigen::RowVector3d::Zero(); //= mean(B);
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

    //calculate the mean
    for (int i=0;i<startP.rows();i++)
    {	centroid_startP=centroid_startP+startP.row(i);
        centroid_finalP=centroid_finalP+finalP.row(i);
    }

    centroid_startP=centroid_startP/startP.rows();
    centroid_finalP=centroid_finalP/startP.rows();

    for (int i=0;i<startP.rows();i++)
        H=H+(startP.row(i)-centroid_startP).transpose()*(finalP.row(i)-centroid_finalP);

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    if (V.determinant()<0)
        V.col(2)=-V.col(2)*(-1);

    Eigen::MatrixXd R=V*U.transpose();

    Eigen::Matrix4d C_A = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d C_B = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d R_new = Eigen::Matrix4d::Identity();

    C_A.block<3,1>(0,3)=-centroid_startP.transpose();
    R_new.block<3,3>(0,0)=R;

    C_B.block<3,1>(0,3)=centroid_finalP.transpose();


    transf = C_B * R_new * C_A;

    //std::cout<<"trans: "<<transf<<std::endl;

    Eigen::Quaterniond mat_rot(transf.block<3,3>(0,0));

    Eigen::Vector3d trasl=transf.block<3,1>(0,3).transpose();

    transfParameters<<trasl(0),trasl(1),trasl(2),mat_rot.x(),mat_rot.y(),mat_rot.z(),mat_rot.w();

}

} // namespace package_name

int main(int argc, char **argv)
{
    ros::init(argc, argv, "online_tf");

    ROS_ERROR("Nargs %d val=%s e % s", argc,argv[0],argv[1]);

    ros::NodeHandle nh;

    transformations::online_tf node(nh,argc,argv);

    while(ros::ok())
    {

    ros::spinOnce();
    }

    return 0;
}

main()
{
    F_red=[0,14,28,42;0,7,14,21];
    F_green=[14,28,42,0;0,7,14,21];
    F_black=[28,42,0,14;0,7,14,21];
    F_blue=[42,0,14,28;0,7,14,21];
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    vis_pub.publish( marker );


    /*car model*/

    a(t) = 0;
       v(t) = v(t-1) + Ts*a(t);
       dx(t) = v(t)  *cos(phi(t-1));
       dy(t) = v(t)  *sin(phi(t-1));
       dphi(t) =  (v(t)/L)*sin(theta(t-1));     %dphi(t) =   u(t-1);
       dtheta(t) = u(t-1);

       x(t) = x(t-1) + Ts*dx(t);
       y(t) = y(t-1) + Ts*dy(t);
       phi(t) = phi(t-1) + Ts*dphi(t);
       theta(t) = theta(t-1) + Ts*dtheta(t);

       if phi(t) > 0
           phi(t) = mod(phi(t), 2*pi);
       end

       if phi(t) < 0
           phi(t) = mod(phi(t), -2*pi);
       end

       if phi(t)> pi
           phi(t)= phi(t) - 2*pi;
       end

       if phi(t)< -pi
           phi(t)= phi(t) + 2*pi;
       end

       mapxy = [x(t),y(t)];
}



