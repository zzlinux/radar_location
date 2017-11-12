#ifndef HITCRT_UTILITY_H_
#define HITCRT_UTILITY_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <stddef.h>

namespace hitcrt
{
union Float2uchar
{
    float fl;
    unsigned char ch[4];
};

void find_closest_point(std::vector<cv::Point2f>& center_point, int cols, cv::Point2f& pt);



}

#endif // HITCRT_UTILITY_H_
