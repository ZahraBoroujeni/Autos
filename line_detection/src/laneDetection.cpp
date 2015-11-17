#include "laneDetection.h"


using namespace std;
using namespace cv;

//#define PAINT_OUTPUT
#define PROJECTED_IMAGE_HEIGTH 160

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;

cLaneDetection::cLaneDetection(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"),detector(16,Point(0,40),Point(200,PROJECTED_IMAGE_HEIGTH)),model(true)
{
    //m_Busy = false;
    //priv_nh_.param<std::string>("PATH_2FEATURES", PATH_2FEATURES, "");
    //priv_nh_.param<std::string>("PATH_30FEATURES", PATH_30FEATURES, "");
    m_LastValue = 0;
    read_images_ = nh.subscribe(nh_.resolveName("/camera/ground_image"), 1,&cLaneDetection::ProcessInput,this);
}

cLaneDetection::~cLaneDetection()
{
}

int cLaneDetection::Init()
{
	//firstFrame = True;
	imagecount = 0;
	m_LastValue = 0;
	return 1;
}

//re-initialize the whole system
void cLaneDetection::resetSystem()
{
    //model = LaneModel(false);
}

void cLaneDetection::ProcessInput(const sensor_msgs::Image::ConstPtr& msg)
{
    // VideoInput
    //std::cout << "Hey, listen!" << std::endl;
    //ROS_INFO("CERTAINTY:");
    //ros::Time begin = ros::Time::now();

    try
    {
        ros::WallTime begin = ros::WallTime::now();

        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        
        Mat image;
        image = cv_ptr->image.clone();
        //row:480, col:640
        // Mat transformedImage = image(Rect(0,300,200,PROJECTED_IMAGE_HEIGTH)).clone();
        // Mat sobeledImage     = image(Rect(0,200,200,PROJECTED_IMAGE_HEIGTH)).clone();
        // Mat groundPlane      = image(Rect(0,100,200,PROJECTED_IMAGE_HEIGTH)).clone();

        Mat transformedImage = image(Rect(220,20,200,PROJECTED_IMAGE_HEIGTH)).clone();
        Mat sobeledImage     = image(Rect(220,20,200,PROJECTED_IMAGE_HEIGTH)).clone();
        Mat groundPlane      = image(Rect(220,20,200,PROJECTED_IMAGE_HEIGTH)).clone();

        

        //create an output image for debugging
        //Mat generalOutputImage(300,800,CV_8UC3,Scalar(0,0,0));
        vector<Point2d> laneMarkings = detector.detect(transformedImage,sobeledImage,groundPlane);

        #ifdef PAINT_OUTPUT
            Mat transformedImagePaintable = transformedImage.clone();
            cvtColor(transformedImagePaintable,transformedImagePaintable,CV_GRAY2BGR);
            for(int i = 0;i < (int)laneMarkings.size();i++)
            {
                circle(transformedImagePaintable,laneMarkings.at(i),1,Scalar(0,0,255),-1);
            }

            Point2d p1(4,160-1);
            Point2d p2(195,160-1);
            Point2d p3(126,50);
            Point2d p4(74,50);
            line(transformedImagePaintable,p1,p2,Scalar(0,200,0));
            line(transformedImagePaintable,p2,p3,Scalar(0,200,0));
            line(transformedImagePaintable,p3,p4,Scalar(0,200,0));
            line(transformedImagePaintable,p4,p1,Scalar(0,200,0));
            cv::imshow("foo", transformedImagePaintable);
            cv::imshow("foo1", image);
            cv::waitKey(1);
        #endif
        //use the detected lane markings to find contours in the image
        Mat circleImage(PROJECTED_IMAGE_HEIGTH,200,CV_8UC1,Scalar(0));
        for(int i = 0;i < (int)laneMarkings.size();i++)
        {
            circle(circleImage,laneMarkings.at(i),3,Scalar(255),-1);
        }

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(circleImage,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(0,0));
        ContourModel cModel;
        bool midLaneFound = cModel.update(contours,laneMarkings);
        vector<vector<Point2d> > nicelyGroupedPoints = cModel.points;

        
        #ifdef PAINT_OUTPUT
                //---------------------- DEBUG OUTPUT CONTOURS ---------------------------------//
            Mat contourImagePaintable = transformedImage.clone();
            cvtColor(contourImagePaintable,contourImagePaintable,CV_GRAY2BGR);
            //draw the detected contours on the output image
            for(int i = 0;i < (int)cModel.points.size();i++)
            {
                Scalar color(255,255,255);
                if(i == 0)color = Scalar(0,0,255);if(i == 1)color = Scalar(0,255,0);if(i == 2)color = Scalar(255,0,0);
                if(i == 3)color = Scalar(255,255,0);if(i == 4)color = Scalar(255,0,255);if(i == 5)color = Scalar(255,255,0);

                vector<Point2d> pointGroup = cModel.points.at(i);
                for(int j = 0;j < (int)pointGroup.size();j++)
                {
                    Point2d currP = pointGroup.at(j);
                    currP.x += 100;
                    circle(contourImagePaintable,currP,1,color,-1);
                }
            }
            cv::imshow("foo2", contourImagePaintable);

            //---------------------- END DEBUG OUTPUT CONTOURS------------------------------//
        #endif
       //ROS_ERROR("Found: %d",midLaneFound);
        model.improvedUpdate(&nicelyGroupedPoints,midLaneFound);
        //ROS_ERROR("CERTAINTY: %d",model.certainty);
        // SEND OUT OLD LANE INFORMATION
        double curvature; //1/cm
        double distanceRightLane; //cm
        double angle; //rad
        bool isCurve;
        model.getCarState(&curvature,&distanceRightLane,&angle,&isCurve);
        //send is curve info:
        int stamp = 0;
        ros::WallTime end = ros::WallTime::now();
        ros::WallDuration d= end-begin;
        ROS_ERROR("time: %ld", d.toNSec()/1000000); 


        // ros::Time end = ros::Time::now();
        // ROS_ERROR("time: %d", ((end.nsec-begin.nsec)/1000000)); 
        #ifdef PAINT_OUTPUT
            //---------------------- DEBUG OUTPUT LANE MODEL---------------------------------//
            int carOffset = 50;
            Mat laneModelDrawing(PROJECTED_IMAGE_HEIGTH+carOffset,200,CV_8UC3,Scalar(0,0,0));
            Mat transformedImageCopy = transformedImage.clone();
            cvtColor(transformedImageCopy,transformedImageCopy,CV_GRAY2BGR);
            transformedImageCopy.copyTo(laneModelDrawing(Rect(0,carOffset,transformedImageCopy.cols,transformedImageCopy.rows)));
            model.getDebugImage(laneModelDrawing);
            cv::imshow("foo3", laneModelDrawing);

           //---------------------- END DEBUG OUTPUT LANE MODEL------------------------------//
        #endif

    } 
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cLaneDetection");
    ros::NodeHandle nh;
    cLaneDetection node(nh);
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}