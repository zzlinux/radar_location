#include "utility.h"
#include <opencv2/core/core.hpp>
namespace hitcrt
{
void find_closest_point(std::vector<cv::Point2f>& center_point, int cols, cv::Point2f& pt)
{
    int closest_u = cols*10;
    for(size_t i = 0;i < center_point.size();i++)
    {
        if(std::abs(center_point[i].x-cols/2) < closest_u)
        {
            closest_u = std::abs(center_point[i].x-cols/2);
            pt = center_point[i];
        }
    }
}










}
