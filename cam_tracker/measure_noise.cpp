#include <algorithm>
#include <getopt.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>

#include "util.h"

using namespace cv;
using namespace std;

static const uchar R_INDEX = 2;
static const uchar G_INDEX = 1;
static const uchar B_INDEX = 0;

static Scalar NOISE_MULTIPLIER;

static struct option long_options[] = {
    { "help", no_argument, 0, 'h' },
    { "red-multiplier", required_argument, 0, 'r' },
    { "green-multiplier", required_argument, 0, 'g' },
    { "blue-multiplier", required_argument, 0, 'b' },
    { "bilateral-filter", no_argument, 0, 'B' },
    { 0, 0, 0, 0 }
};

static const char *USAGE =
"Usage: measure_noise [OPTIONS] URI\n"
"Measure and display the noise from a video\n"
"\n"
"Mandatory arguments to long options are mandatory for short options too.\n"
"  -h, --help                  Display this help and exit\n"
"  -r, --red-multiplier=INT    Multiply noise in red channel by INT\n"
"  -g, --green-multiplier=INT  Multiply noise in green channel by INT\n"
"  -b, --blue-multiplier=INT   Multiply noise in blue channel by INT\n"
"  -B, --bilateral-filter      Apply a bilateral filter\n"
"\n"
"Noise statistics are calculated before the INT multipliers are applied.\n"
"The multipliers only affect the noise displayed in the output window.\n"
"\n"
"The bilateral filter, if applied, affects both the statistics calculated\n"
"and the noise displayed in the output window.\n"
"\n"
"Example URIs:\n"
"  v4l2:///dev/video0                           V4L2 capture device\n"
"  video.avi                                    AVI video file\n"
"  http://192.168.0.8:8080/video?format=.mjpeg  HTTP MJPEG stream\n"
;

void usage_and_exit(FILE *stream, int ret_val)
{
    fprintf(stream, "%s", USAGE);
    exit(ret_val);
}

int main(int argc, char **argv)
{
    NOISE_MULTIPLIER[R_INDEX] = 1;
    NOISE_MULTIPLIER[G_INDEX] = 1;
    NOISE_MULTIPLIER[B_INDEX] = 1;

    int option_index = 0;
    int opt;
    int filter = 0;
    while ((opt = getopt_long(argc, argv, "r:g:b:B", long_options, &option_index)) != -1) {
        switch (opt) {
            case 'h': usage_and_exit(stdout, EXIT_SUCCESS);
            case 'r': NOISE_MULTIPLIER[R_INDEX] = atoi(optarg); break;
            case 'g': NOISE_MULTIPLIER[G_INDEX] = atoi(optarg); break;
            case 'b': NOISE_MULTIPLIER[B_INDEX] = atoi(optarg); break;
            case 'B': filter = 1; break;
            default: usage_and_exit(stderr, EXIT_FAILURE);
        }
    }
    if (optind != argc - 1) {
        usage_and_exit(stderr, EXIT_FAILURE);
    }

    VideoCapture capture = open_stream(argv[optind]);
    Ptr<Mat> prev_frame;
    Ptr<Mat> curr_frame(new Mat());
    int key = -1;
    static const char *NOISE_WINDOW = "Noise Window";
    static const char *OUTPUT_WINDOW = "Output Window";
    namedWindow(NOISE_WINDOW, WINDOW_AUTOSIZE);
    namedWindow(OUTPUT_WINDOW, WINDOW_AUTOSIZE);

    while (key != 'q' && capture.read(*curr_frame)) {
        if (filter) {
            Ptr<Mat> unfiltered_frame = curr_frame;
            curr_frame.reset(new Mat());
            bilateralFilter(*unfiltered_frame, *curr_frame, 15, 80, 80);
        }
        if (prev_frame != NULL) {
            Mat noise_frame;
            absdiff(*curr_frame, *prev_frame, noise_frame);
            Scalar max, mean, stddev;
            meanStdDev(noise_frame, mean, stddev);
            for (int y = 0; y < noise_frame.rows; ++y) {
                for (int x = 0; x < noise_frame.cols; ++x) {
                    for (int c = 0; c < 3; ++c) {
                        max.val[c] = std::max(max.val[c], static_cast<double>(noise_frame.at<uchar>(y, x * 3 + c)));
                        noise_frame.at<uchar>(y, x * 3 + c) =
                            std::min<uchar>(255, noise_frame.at<uchar>(y, x * 3 + c) * NOISE_MULTIPLIER[c]);
                    }
                }
            }
            fprintf(stderr,
                    "\rR[mean:%6.2f, stddev:%9.5f, max:%3.0f], G[mean:%6.2f, stddev:%9.5f, max:%3.0f], B[mean:%6.2f, stddev:%9.5f, max:%3.0f]",
                    mean.val[2], stddev.val[2], max.val[2],
                    mean.val[1], stddev.val[1], max.val[1],
                    mean.val[0], stddev.val[0], max.val[0]);
            imshow(NOISE_WINDOW, noise_frame);
            imshow(OUTPUT_WINDOW, *curr_frame);
            key = waitKey(1000 / 6);
        }
        prev_frame = curr_frame;
        curr_frame.reset(new Mat());
    }
    fprintf(stderr, "\n");
}

