#include <map>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace cv;

int distance(Vec3b a, Vec3b b)
{
    int d0 = a[0] - b[0];
    int d1 = a[1] - b[1];
    int d2 = a[2] - b[2];
    return sqrt(d0 * d0 + d1 * d1 + d2 * d2);
}

int main(int argc, char **argv)
{
    //VideoCapture input(0);
    VideoCapture input("elmo.avi");
    if (!input.isOpened()) {
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

    /*
    Mat hsvFrame;
    cvtColor(rgbFrame, hsvFrame, CV_BGR2HSV);

//    std::map<int, int> hue2count;
    Vec3b yellow(0, 0xff, 0xff);
    Vec3b black(0, 0, 0);
    for (int i = 0; i < hsvFrame.rows; ++i) {
        for (int j = 0; j < hsvFrame.cols; ++j) {
            Vec3b point = hsvFrame.at<Vec3b>(i, j);
            int hue = point[0];
            if (hue >= 10 && hue <= 40) {
                rgbFrame.at<Vec3b>(i, j) = yellow;
            } else {
                rgbFrame.at<Vec3b>(i, j) = black;
            }
 //           ++hue2count[point[0]];
            //fprintf(stderr, "(%d, %d, %d)\n", point[0], point[1], point[2]);
        }
    }

  //  for (int i = 0; i <= 360; ++i) {
  //      printf("hue2count[%d] = %d\n", i, hue2count[i]);
  //  }
    */
    output << rgbFrame;

    return 0;
}

