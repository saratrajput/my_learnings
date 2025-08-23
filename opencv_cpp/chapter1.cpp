#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <iostream>


using namespace std;

// Importing images
void main()
{
    string path = "groovy_gorilla.jpg"
    cv::Mat img = cv::imread(path);
    cv::imshow("Image", img);
    cv::waitKey(0);
}