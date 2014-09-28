#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "util.h"

using namespace cv;

static const char *USAGE = "\
Usage: calibrate_color_filter using URI\n\
\n\
URI must be an MJPEG stream.\n";

int main(int argc, char **argv)
{
    if (argc != 3 || strcmp(argv[1], "using")) {
        fprintf(stderr, "%s", USAGE);
        exit(1);
    }

    const char *MASK_WINDOW = "Mask";
    const char *OUTPUT_WINDOW = "Output";
    VideoCapture capture = open_stream(argv[2]);
    namedWindow(MASK_WINDOW, WINDOW_AUTOSIZE);
    namedWindow(OUTPUT_WINDOW, WINDOW_AUTOSIZE);

    int h_min = 23, h_max = 70;
    int s_min = 85, s_max = 133;
    int v_min = 182, v_max = 205;
    createTrackbar("H min", OUTPUT_WINDOW, &h_min, 179);
    createTrackbar("H max", OUTPUT_WINDOW, &h_max, 179);
    createTrackbar("S min", OUTPUT_WINDOW, &s_min, 255);
    createTrackbar("S max", OUTPUT_WINDOW, &s_max, 255);
    createTrackbar("V min", OUTPUT_WINDOW, &v_min, 255);
    createTrackbar("V max", OUTPUT_WINDOW, &v_max, 255);

    Mat bgr_frame;
    capture.read(bgr_frame);

    int key = -1;
    while (key != 'q') {
        if (key != -1) {
            capture.read(bgr_frame);
        }
        Mat hsv_frame;
        cvtColor(bgr_frame, hsv_frame, CV_BGR2HSV);
        Mat mask, filtered;
        inRange(hsv_frame,
                Scalar(h_min, s_min, v_min),
                Scalar(h_max, s_max, v_max),
                mask);
        bitwise_and(bgr_frame, bgr_frame, filtered, mask);
        imshow(MASK_WINDOW, filtered);
        imshow(OUTPUT_WINDOW, bgr_frame);
        key = waitKey(1);
    }
}

