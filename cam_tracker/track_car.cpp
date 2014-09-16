#include <algorithm>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>

#include "util.h"

using namespace cv;
using namespace std;

Point2f find_center(Mat image, const vector<KeyPoint> &keypoints)
{
    float min_y = image.rows, min_x = image.cols;
    float max_y = 0, max_x = 0;
    for (vector<KeyPoint>::const_iterator it = keypoints.begin(); it != keypoints.end(); ++it) {
        min_x = std::min(min_x, it->pt.x);
        max_x = std::max(max_x, it->pt.x);
        min_y = std::min(min_y, it->pt.y);
        max_y = std::max(max_y, it->pt.y);
    }
    Point2f center((min_x + max_x) / 2, (min_y + max_y) / 2);
    printf("%f, %f\n", center.x, center.y);
    return center;
}

Mat find_homography(vector<DMatch> good_matches,
        vector<KeyPoint> training_image_keypoints,
        vector<KeyPoint> query_image_keypoints)
{
    vector<Point2f> query_pts;
    vector<Point2f> train_pts;
    for (vector<DMatch>::const_iterator it = good_matches.begin(); it != good_matches.end(); ++it) {
        query_pts.push_back(training_image_keypoints[it->queryIdx].pt);
        train_pts.push_back(query_image_keypoints[it->trainIdx].pt);
    }
    return findHomography(Mat(query_pts), Mat(train_pts), CV_RANSAC, 5.0);
}

int main(int argc, char **argv)
{
    SIFT sift;
    BFMatcher matcher;

    Mat training_image = imread("car.png", 0);
    vector<KeyPoint> training_image_keypoints;
    sift.detect(training_image, training_image_keypoints, Mat());
    printf("training image # kps: %lu\n", training_image_keypoints.size());
    Mat training_image_descriptors;
    sift.compute(training_image, training_image_keypoints, training_image_descriptors);
    Point2f training_center =
        find_center(training_image, training_image_keypoints);

    VideoCapture capture = open_stream(argv[1]);
    Mat bgr_frame;
    int key = -1;
    static const char *OUTPUT_WINDOW = "Output Window";
    namedWindow(OUTPUT_WINDOW, WINDOW_AUTOSIZE);
    const Scalar RED(0, 0, 255);
    while (key == -1 && capture.read(bgr_frame)) {
        Mat query_image;
        cvtColor(bgr_frame, query_image, CV_BGR2GRAY);
        vector<KeyPoint> query_image_keypoints;

        Mat mask = Mat::zeros(query_image.size(), CV_8U);
        Mat roi(mask, cv::Rect(390, 250, 100, 60));
        roi = Scalar(255, 255, 255);
        rectangle(bgr_frame, Point(390, 250), Point(490, 310), RED);

        sift.detect(query_image, query_image_keypoints, mask);
        printf("query image # kps: %3lu", query_image_keypoints.size());
        if (query_image_keypoints.size() == 0) {
            printf("\n");
            imshow(OUTPUT_WINDOW, bgr_frame);
            continue;
        }
        Mat query_image_descriptors;
        sift.compute(query_image, query_image_keypoints, query_image_descriptors);

        vector<vector<DMatch> > matches;
        matcher.knnMatch(training_image_descriptors,
                query_image_descriptors,
                matches,
                2);
        vector<DMatch> good_matches;
        printf(", # of matches: %3lu", matches.size());
        for (unsigned i = 0; i < matches.size(); ++i) {
            if (matches[i][0].distance < .75 * matches[i][1].distance) {
                good_matches.push_back(matches[i][0]);
            }
        }
        printf(", # of good matches: %3lu", good_matches.size());
        if (good_matches.size() < 4) { // 4 is minimum needed by findHomography
            printf("\n");
            imshow(OUTPUT_WINDOW, bgr_frame);
            continue;
        }

        Mat M = find_homography(good_matches, training_image_keypoints, query_image_keypoints);
        vector<Point2f> training_points;
        training_points.push_back(training_center);
        vector<Point2f> query_points;
        perspectiveTransform(training_points, query_points, M);
        printf(", car center: %f, %f\n", query_points[0].x, query_points[0].y);

        circle(bgr_frame, Point2i(int(query_points[0].x), int(query_points[0].y)), 2, RED, -1);
        imshow(OUTPUT_WINDOW, bgr_frame);
        key = waitKey(1);
    }
}

