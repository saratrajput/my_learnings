#include <opencv2/opencv.hpp>

using namespace cv;

int main()
{
    Mat image;
    image = imread("sample.jpeg");

    namedWindow("Display image", WINDOW_AUTOSIZE);
    imshow("Display image", image);

    waitKey(0);

    return 0;
}