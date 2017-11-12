//
// Created by robocon on 11/6/17.
//

#ifndef RADAR_LOCATION_RADAR_H
#define RADAR_LOCATION_RADAR_H
//urg
#include "Urg_driver.h"
#include "Connection_information.h"
#include "math_utilities.h"

#include "serialapp.h"
#include <vector>
#include "Algorithm.h"
#include <opencv2/core/core.hpp>
using namespace std;
using namespace qrk;
using namespace cv;
namespace hitcrt
{
#define  RANDOMPOINT VecPoint.at(leftPointsIndex.at(rand()%leftPointsIndex.size()))
#define RANDCOLOR Scalar(rand()/256,rand()/256,rand()/256)
class Radar {
private:
    Urg_driver urg;
    SerialApp serial;
    double maxDisInXY;
public:
    Radar();
    void init();
    int run();
    ~Radar();
private:
    void depth2xy(const vector<long>&data,vector<POINT> &VecPoint);
    void MyRansc(vector<POINT> & VecPoint,vector<vector<POINT>> &lines);
    void myimshow(vector<POINT> points);
    bool sortLines(vector<int> & l1,vector<int> & l2);
    POINT world2pixel(POINT old);
    void fillMat(Mat & img,vector<POINT> &points,Scalar color);
    void getRadarPosition(vector<vector<POINT>>&lines);
    vector<POINT> KalmanFiltePoints(const vector<POINT> VecPoint);
};
}


#endif //RADAR_LOCATION_RADAR_H
