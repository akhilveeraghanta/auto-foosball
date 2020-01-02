#include "opencv2/core.hpp"
#include "ros/ros.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "urboi");
    Mat img(100, 100, CV_8UC3);
    randu(img, Scalar(0, 0, 0), Scalar(255, 255, 255));

    imshow("random colors image", img);
    waitKey();

    return 0;
}
