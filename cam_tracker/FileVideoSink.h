#ifndef FILE_VIDEO_SINK_H
#define FILE_VIDEO_SINK_H

#include "VideoSink.h"
#include <opencv2/highgui/highgui.hpp>

class FileVideoSink : public VideoSink {
    public:
        FileVideoSink(const char *fname);
        void sinkFrame(cv::Mat frame);

    private:
        cv::VideoWriter output;
};

#endif

