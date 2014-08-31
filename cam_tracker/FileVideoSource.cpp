#include "FileVideoSource.h"

using namespace cv;

FileVideoSource::FileVideoSource(const char *fname) : input(fname)
{
}

Mat FileVideoSource::sourceFrame()
{
    Mat frame;
    return frame;
}

