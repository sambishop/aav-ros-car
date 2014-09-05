#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

bool is_white(Mat image, int x, int y)
{
    Vec3b point = image.at<Vec3b>(y, x);
    return point[0] >= 244 && point[1] >= 244 && point[2] >= 244;
}

int read_digit(Mat image, int position)
{
    int base_x = position * 12;
    if (is_white(image, base_x + 8, 12)) {
        return 2;
    } else if (is_white(image, base_x + 6, 2)) {
        return 4;
    } else if (is_white(image, base_x + 4, 2)) {
        return 1;
    } else if (is_white(image, base_x + 0, 0)
            && !is_white(image, base_x + 4, 8)) {
        return 5;
    } else if (is_white(image, base_x + 2, 2)) {
        return 6;
    } else if (is_white(image, base_x + 0, 0)
            && !is_white(image, base_x + 2, 13)) {
        return 7;
    } else if (is_white(image, base_x + 2, 8)
            && !is_white(image, base_x + 5, 8)) {
        return 0;
    } else if (is_white(image, base_x + 8, 6)
            && !is_white(image, base_x + 0, 7)) {
        return 9;
    } else if (is_white(image, base_x + 3, 6)) {
        return 8;
    }
    return 3;
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
        return -1;
    }

    Mat image;
    image = imread(argv[1], CV_LOAD_IMAGE_COLOR);

    if (!image.data) {
        cout <<  "Could not open or find the image" << std::endl;
        return -1;
    }

    int month = read_digit(image, 0) * 10 + read_digit(image, 1);
    int day = read_digit(image, 3) * 10 + read_digit(image, 4);
    int year = 2000 + read_digit(image, 6) * 10 + read_digit(image, 7);

    int hour = read_digit(image, 9) * 10 + read_digit(image, 10);
    int minute = read_digit(image, 12) * 10 + read_digit(image, 13);
    int second = read_digit(image, 15) * 10 + read_digit(image, 16);
    printf("%02d/%02d/%02d %02d:%02d:%02d\n",
            month, day, year, hour, minute, second);
    return 0;
}

