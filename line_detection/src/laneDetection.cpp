/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/


/**
 *
 *AADC_Spurerkennung
 *
 *BRICKL_CHRISTOPH, AEV
 */

#include "laneDetection.h"

using namespace std;
using namespace cv;

#define PAINT_OUTPUT
#define PROJECTED_IMAGE_HEIGTH 160

static const uint32_t MY_ROS_QUEUE_SIZE = 1000;
//,detector(16,Point(0,40),Point(200,PROJECTED_IMAGE_HEIGTH)),model(true)

cLaneDetection::cLaneDetection(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"),detector(16,Point(0,40),Point(200,PROJECTED_IMAGE_HEIGTH)),model(true)
{
    //m_Busy = false;
    m_LastValue = 0;
    read_images_ = nh.subscribe(nh_.resolveName("/camera/rgb/image_rect_color"), 1000,&cLaneDetection::ProcessInput,this);
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
    model = LaneModel(false);
}

void cLaneDetection::ProcessInput(const sensor_msgs::Image::ConstPtr& msg)
{
    // VideoInput
    std::cout << "Hey, listen!" << std::endl;
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
          //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          //cv_ptr = cv_bridge::toCvShare(msg);

        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
        
        Mat image;
        image = cv_ptr->image.clone();
        //row:480, col:640
        Mat transformedImage = image(Rect(170,300,200,PROJECTED_IMAGE_HEIGTH)).clone();
        Mat sobeledImage     = image(Rect(170,200,200,PROJECTED_IMAGE_HEIGTH)).clone();
        Mat groundPlane      = image(Rect(170,100,200,PROJECTED_IMAGE_HEIGTH)).clone();
        

        //create an output image for debugging
        Mat generalOutputImage(300,800,CV_8UC3,Scalar(0,0,0));
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
         // LOG_INFO(cString::Format("Found: %d",midLaneFound));
        model.improvedUpdate(&nicelyGroupedPoints,midLaneFound);

        // LOG_INFO(cString::Format("CERTAINTY: %d",model.certainty));


        // SEND OUT OLD LANE INFORMATION
        double curvature; //1/cm
        double distanceRightLane; //cm
        double angle; //rad
        bool isCurve;
        model.getCarState(&curvature,&distanceRightLane,&angle,&isCurve);
        //SendLaneInfo(curvature, distanceRightLane-1, angle, 0);

        //send is curve info:
        int stamp = 0;
        //SendSignalValueMessage(&m_oIsCurvePin, isCurve, stamp);


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
    catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }




       
/*

        #ifdef PAINT_OUTPUT
        cObjectPtr<IMediaSample> pNewRGBSample;
        if (IS_OK(AllocMediaSample(&pNewRGBSample)))
        {
            tTimeStamp tmStreamTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
            pNewRGBSample->Update(tmStreamTime, generalOutputImage.data, tInt32(3*800*300), 0);
            m_oVideoOutputPin.Transmit(pNewRGBSample);
        }
        #endif
    }*/
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cLaneDetection");
        
   //ROS_ERROR("Nargs %d val=%s e % s", argc,argv[0],argv[1]);

    ros::NodeHandle nh;

    cLaneDetection node(nh);

    while(ros::ok())
    {
 
    ros::spinOnce();
    }

    return 0;
}