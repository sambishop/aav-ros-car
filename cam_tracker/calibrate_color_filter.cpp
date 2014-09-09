#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace cv;

static const char *USAGE = "\
Usage: calibrate_color_filter using URL\n\
\n\
URL must be an MJPEG stream.\n";

static const char *OUTPUT_WINDOW = "Color Calibration";

VideoCapture open_camera(const char *url)
{
    VideoCapture capture;
    if (!capture.open(url)) {
        fprintf(stderr, "error opening video stream\n");
        exit(1);
    }
    return capture;
}

int main(int argc, char **argv)
{
    if (argc != 3 || strcmp(argv[1], "using")) {
        fprintf(stderr, "%s", USAGE);
        exit(1);
    }

    VideoCapture capture = open_camera(argv[2]);
    namedWindow(OUTPUT_WINDOW, WINDOW_AUTOSIZE);
    Mat bgr_frame;
    int key = -1;
    while (key == -1 && capture.read(bgr_frame)) {
        Mat hsv_frame;
        cvtColor(bgr_frame, hsv_frame, CV_BGR2HSV);
        Mat mask, filtered;
        inRange(hsv_frame,
                // TODO replace these with trackbar inputs
                Scalar(5, 0, 0),
                Scalar(15, 255, 255),
                mask);
        bitwise_and(bgr_frame, bgr_frame, filtered, mask);
        imshow(OUTPUT_WINDOW, filtered);
        key = waitKey(1);
    }
}

