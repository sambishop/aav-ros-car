#ifndef NULL_VIDEO_SINK_H
#define NULL_VIDEO_SINK_H

#include "VideoSink.h"

class NullVideoSink : public VideoSink {
    public:
        void sinkFrame(cv::Mat frame);
};

#endif

