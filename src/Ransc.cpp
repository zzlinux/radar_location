#include "Ransc.h"
#include <cfloat>
#include <cmath>
#include <ctime>
#include <cstdio>
#include <algorithm>
#include <iostream>
#include <fstream>

using namespace std;

#define RAND_DATA  rand()%(rand_end - rand_begin+1)+rand_begin
#define RAND_POINT  P.at(RAND_DATA);

void RANSC(const vector<POINT> P, size_t nrIterations, vector<POINT> & best_inliners)
{
    double bestError_sum = DBL_MAX;
    LINE  best_solution1 = {0};
    LINE  best_solution2 = {0};
    LINE best_solution3 = {0};
    double delta = 100;//threshold, test by real data

    // RANSAC
    size_t nrPoints = P.size();
    srand((unsigned)time(NULL));//get random data
    unsigned int rand_begin;
    unsigned int rand_end;

    for (size_t k = 0;k<nrIterations;k++)
    {
        size_t inliner_num = 0;

        //get sample points
        LINE line1;
        LINE line2;
        LINE line3;

        rand_begin = 0;
        rand_end = floor(nrPoints/3);
        line1.PointStart = RAND_POINT;
        line1.PointEnd   = RAND_POINT;

        rand_begin = rand_end;
        rand_end = 2*floor(nrPoints/3);
        line2.PointStart = RAND_POINT;
        line2.PointEnd   = RAND_POINT;

        rand_begin = rand_end;
        rand_end = nrPoints-1;
        line3.PointStart = RAND_POINT;
        line3.PointEnd   = RAND_POINT;

        double error_sum = 0;

        for (size_t i = 0; i<nrPoints;i++)
        {
               POINT point = P.at(i);

              // Find the distance to the line
              double dist1 = CalPointLineDistance(point,line1);
              double dist2 = CalPointLineDistance(point,line2);
              double dist3 = CalPointLineDistance(point,line3);
              double distmin = std::min(std::min(dist1, dist2), dist3);
              if (distmin < delta)
              {
                  error_sum = error_sum + distmin;
                  inliner_num = inliner_num+1;
              }
        }


        if ( error_sum < bestError_sum && inliner_num*1.0f / nrPoints > 0.90)
        {
            bestError_sum = error_sum;
            best_solution1 = line1;
            best_solution2 = line2;
            best_solution3 = line3;
        }
    }

    for (size_t i = 0; i<nrPoints;i++)
    {
          POINT point = P.at(i);

          //Find the distance to the line
          double dist1 = CalPointLineDistance(point,best_solution1);
          double dist2 = CalPointLineDistance(point,best_solution2);
          double dist3 = CalPointLineDistance(point,best_solution3);

          double distmin = std::min(std::min(dist1, dist2), dist3);
          if (distmin > delta)
              continue;

          best_inliners.push_back(point);
    }
}

void KalmanFiltePoints(const vector<POINT> VecPoint, vector<POINT>  &FilterPoint)
{
    POINT point;
    kalman1_state *x_filter = new kalman1_state;//kalman滤波初始化
    kalman1_state *y_filter = new kalman1_state;

    kalman1_init(x_filter, 3, 100);//4 500
    kalman1_init(y_filter, 3,100);

    for(size_t i=0;i<VecPoint.size();i++)
    {
        point.x = kalman1_filter(x_filter, VecPoint.at(i).x);//Kalman 滤波
        point.y = kalman1_filter(y_filter, VecPoint.at(i).y);
        FilterPoint.push_back(point);
    }
    delete x_filter;
    delete y_filter;
}

void RadarLocation(const vector<POINT>VecPoint, RADAR_INFO &stRadarInfo)
{
    vector<POINT> FilterPoint;
    KalmanFiltePoints(VecPoint,FilterPoint);//radar cordinates kalman filter

    vector<double> PointCurvature;
    CalVecCurvature(FilterPoint, PointCurvature);//计算各点曲率

    vector<double> PointCurvatureFilterData;
    GlideFilter(PointCurvature, PointCurvatureFilterData);//曲率滑动平均值滤波

    vector<size_t> CornerIndex;
    CalPeakValIndex(PointCurvatureFilterData, CornerIndex);//找到角点即曲率极大值点的下标

//#if 1 //Debug
//    for(size_t i=0;i<CornerIndex.size();i++)
//        cout<<"CornerIndex"<<i <<" =    "<<CornerIndex.at(i)<<endl;
//#endif // 0

    RADAR_CORDINATES RadarCordinates;
    CalRadarCordinates(FilterPoint, CornerIndex, RadarCordinates);//计算雷达中心坐标
//    cout<<"RadarCordinates.x =  "<<RadarCordinates.x<<endl;
//    cout<<"RadarCordinates.y =  "<<RadarCordinates.y<<endl;
    stRadarInfo.CornerNum = CornerIndex.size();
    stRadarInfo.x = RadarCordinates.x;
    stRadarInfo.y = RadarCordinates.y;

    if(stRadarInfo.CornerNum == 1)
        stRadarInfo.CornerIndex = CornerIndex.front();
    else
        stRadarInfo.CornerIndex = 666;

//    cout << "1" << endl;
}

void WritePointsToFile(const vector<POINT> FilterPoint, const char*  filename)
{
    ofstream output_radar_data;
    output_radar_data.open(filename, ios::out|ios::trunc);
    for (size_t i=0; i<FilterPoint.size();i++)
            output_radar_data<<FilterPoint.at(i).x<<"    "<<FilterPoint.at(i).y<<endl;
    output_radar_data.close();
}

void WriteDataToFile(const vector<double>FilterPoint, const char*  filename)
{
    ofstream output_radar_data;
    output_radar_data.open(filename, ios::out|ios::trunc);
    for (size_t i=0; i<FilterPoint.size();i++)
            output_radar_data<<i<<"    "<<FilterPoint.at(i)<<endl;
    output_radar_data.close();
}

void ReadPointsFromFile(vector<POINT> &FilterPoint, const char*  filename)
{
    ifstream datafile;
    datafile.open(filename, ios::in);

    POINT point;
    while(!datafile.eof())
    {
        datafile>>point.x>>point.y;
        FilterPoint.push_back(point);
    }
    FilterPoint.pop_back();
    datafile.close();
}





