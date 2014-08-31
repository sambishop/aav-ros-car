#ifndef CAMERA_VIDEO_SOURCE_H
#define CAMERA_VIDEO_SOURCE_H

#include "VideoSource.h"
#include <opencv2/highgui/highgui.hpp>

class CameraVideoSource : public VideoSource {
    public:
        CameraVideoSource();
        cv::Mat sourceFrame();

    private:
        cv::VideoCapture input;
};

#endif

