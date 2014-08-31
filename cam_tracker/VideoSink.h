#ifndef VIDEO_SINK_H
#define VIDEO_SINK_H

#include <opencv2/core/core.hpp>

class VideoSink {
    public:
        virtual void sinkFrame(cv::Mat frame) = 0;
};

#endif

