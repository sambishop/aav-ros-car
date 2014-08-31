#ifndef VIDEO_SOURCE_H
#define VIDEO_SOURCE_H

#include <opencv2/core/core.hpp>

class VideoSource {
    public:
        virtual cv::Mat sourceFrame() = 0;
};

#endif

