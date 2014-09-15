#include <algorithm>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <vector>

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
    SiftFeatureDetector detector;
    SiftDescriptorExtractor extractor;
    BruteForceMatcher<L2<float> > matcher;

    Mat training_image = imread("car.png", 0);
    vector<KeyPoint> training_image_keypoints;
    detector.detect(training_image, training_image_keypoints);
    printf("# kps: %d\n", training_image_keypoints.size());
    Mat training_image_descriptors;
    extractor.compute(training_image,
            training_image_keypoints,
            training_image_descriptors);
    Point2f training_center =
        find_center(training_image, training_image_keypoints);

    Mat query_image = imread("car-snapshot.png", 0);
    vector<KeyPoint> query_image_keypoints;
    detector.detect(query_image, query_image_keypoints);
    printf("# kps: %d\n", query_image_keypoints.size());
    Mat query_image_descriptors;
    extractor.compute(query_image,
            query_image_keypoints,
            query_image_descriptors);

    vector<vector<DMatch> > matches;
    matcher.knnMatch(training_image_descriptors,
            query_image_descriptors,
            matches,
            2);
    vector<DMatch> good_matches;
    printf("%d\n", matches.size());
    for (unsigned i = 0; i < matches.size(); ++i) {
        if (matches[i][0].distance < .75 * matches[i][1].distance) {
            good_matches.push_back(matches[i][0]);
        }
    }
    printf("%d\n", good_matches.size());

    Mat M = find_homography(good_matches, training_image_descriptors,
            query_image_descriptors);
    vector<Point2f> training_points;
    training_points.push_back(training_center);
    vector<Point2f> query_points;
    perspectiveTransform(training_points, query_points, M);
    printf("%d\n", query_points.size());
}

