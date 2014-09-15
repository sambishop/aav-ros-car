#include <libavformat/avformat.h>
#include <stdio.h>

#include "util.h"

cv::VideoCapture open_stream(const char *uri)
{
    cv::VideoCapture capture;
    const char *V4L2_PREFIX = "v4l2:///dev/video";
    bool success;
    if (strncmp(uri, V4L2_PREFIX, strlen(V4L2_PREFIX)) == 0) {
        int device = atoi(uri + strlen(V4L2_PREFIX));
        success = capture.open(device);
    } else {
        success = capture.open(uri);
    }
    if (!success) {
        fprintf(stderr, "unable to open %s\n", uri);
        exit(1);
    }
    return capture;
}

