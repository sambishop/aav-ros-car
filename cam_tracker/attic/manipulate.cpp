#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace cv;

int main(int argc, char **argv)
{
    VideoCapture input("elmo.avi");
    if (!input.isOpened()) {
        fprintf(stderr, "open of elmo.avi failed\n");
        return -1;
    }

    Mat rgbFrame;
    input >> rgbFrame;

    VideoWriter output;
    output.open("out.avi", CV_FOURCC('F', 'F', 'V', '1'),
            30, Size(640, 480), true);
    if (!output.isOpened()) {
        fprintf(stderr, "open of output failed\n");
        return -1;
    }

    Mat hsvFrame;
    cvtColor(rgbFrame, hsvFrame, CV_BGR2HSV);

    Vec3b yellow(0, 0xff, 0xff);
    Vec3b black(0, 0, 0);
    for (int i = 0; i < hsvFrame.rows; ++i) {
        for (int j = 0; j < hsvFrame.cols; ++j) {
            Vec3b point = hsvFrame.at<Vec3b>(i, j);
            int hue = point[0];
            int value = point[2];
            if (hue >= 21 && hue <= 36 && value >= 25) {
                rgbFrame.at<Vec3b>(i, j) = yellow;
            } else {
                rgbFrame.at<Vec3b>(i, j) = black;
            }
        }
    }

    output << rgbFrame;

    return 0;
}

