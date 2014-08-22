#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace cv;

int main(int argc, char **argv)
{
    VideoCapture input(0);
    if (!input.isOpened()) {
        return -1;
    }

    Mat rgbFrame;
    input >> rgbFrame;

    VideoWriter output;
    output.open("elmo.avi", CV_FOURCC('F', 'F', 'V', '1'),
            30, Size(640, 480), true);
    if (!output.isOpened()) {
        fprintf(stderr, "open of output failed\n");
        return -1;
    }

    output << rgbFrame;

    return 0;
}

