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
    VideoCapture input(0);
    if (!input.isOpened()) {
        return -1;
    }

    Mat frame;
    input >> frame;

    VideoWriter output;
    output.open("out.avi", CV_FOURCC('F', 'F', 'V', '1'),
            30, Size(640, 480), true);
    if (!output.isOpened()) {
        fprintf(stderr, "open of output failed\n");
        return -1;
    }

    Vec3b yellow(0x00, 0xff, 0xff);
    int min_distance = 1000;
    for (int i = 0; i < frame.rows; ++i) {
        for (int j = 0; j < frame.cols; ++j) {
            Vec3b point = frame.at<Vec3b>(i, j);
            int d = distance(yellow, point);
            if (d < min_distance) {
                fprintf(stderr, "[%d, %d] = (%d, %d, %d), (%d)\n",
                        j, i,
                        point[0], point[1], point[2],
                        d);
                min_distance = d;
            }
        }
    }

    output << frame;

    return 0;
}

