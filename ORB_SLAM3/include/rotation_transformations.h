
#ifndef ROTATION_TRANSFORMATIONS_H
#define ROTATION_TRANSFORMATIONS_H

#include <opencv2/core/core.hpp>
#include <string>

using namespace std;

void getQuaternion(cv::Mat R, float (&Q)[4]);

cv::Mat rot2euler(const cv::Mat & R , string order);
#endif
