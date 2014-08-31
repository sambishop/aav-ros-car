#ifndef FILE_VIDEO_SOURCE_H
#define FILE_VIDEO_SOURCE_H

#include "VideoSource.h"
#include <opencv2/highgui/highgui.hpp>

class FileVideoSource : public VideoSource {
    public:
        FileVideoSource(const char *fname);
        cv::Mat sourceFrame();

    private:
        cv::VideoCapture input;
};

#endif

