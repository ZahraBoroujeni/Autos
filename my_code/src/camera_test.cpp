#include "opencv2/core/core.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

int main(){
cout << "opening device(s)" << endl;

VideoCapture sensor1;sensor1.open(0);
//VideoCapture sensor2;sensor2.open(CV_CAP_OPENNI_ASUS+1);

if( !sensor1.isOpened() ){
    cout << "Can not open capture object 1." << endl;
    return -1;
}
else
{

    Mat depth1,depth2;

    if( !sensor1.grab() ){
        cout << "Sensor1 can not grab images." << endl;
        return -1;
    }else if( sensor1.retrieve( depth1, CV_CAP_OPENNI_DEPTH_MAP ) ) imshow("depth1",depth1);
}
}
