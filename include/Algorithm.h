#ifndef ALGORITHM_H
#define ALGORITHM_H

#include  <stdlib.h>
#include  <iostream>
#include  <valarray>
#include <vector>
#include "BaseRobot.h"
using namespace std;
typedef struct
{
    double x;
    double y;
}POINT;
/*----------------kalman filter------------------------------------------------------------------------*/
/* 1 Dimension */
typedef struct {
    float x;  /* state */
    float A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
    float H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
    float q;  /* process(predict) noise convariance */
    float r;  /* measure noise convariance */
    float p;  /* estimated error convariance */
    float gain;
} kalman1_state;

void kalman1_init(kalman1_state *state, float init_x, float init_p);
float kalman1_filter(kalman1_state *state, float z_measure);
/*----------------kalman filter end------------------------------------------------------------------------*/


/****************************************************/
/**基于曲率的特征分割算法**/
/**参考王永锟学长论文**/
/****************************************************/

typedef struct
{
    float x;
    float y;
    unsigned int CornerIndex;
    unsigned int  CornerNum;
}RADAR_INFO;

typedef struct
{
    POINT PointStart;
    POINT PointEnd;
}LINE;
typedef struct
{
    double k;
    double b;
}LINEKB;
double CalDistance(const POINT &point1, const POINT &point2);
double CalLineLength(LINE &line);
double CalPointLineDistance(const POINT &point, const LINE &line);
double CalCurvature(const std::vector<POINT> &VecPoint);
size_t GetMidIndexofVector(const std::vector<POINT> &VecPoint);

void CalVecCurvature(const std::vector<POINT> &VecPoint, std::vector<double>&PointCurvature);
void CalPeakValIndex(std::vector<double>&PointCurvature, std::vector<size_t>& PeakIndex);

typedef POINT RADAR_CORDINATES;
void CalRadarCordinates(std::vector<POINT>  &VecPoint, std::vector<size_t> &CornerIndex, RADAR_CORDINATES &RadarCordinates);

void CalRadarDirection(LINE & liney,double & direction);
double CalRobotAngle(std::vector<POINT>  &VecPoint);
void GlideFilter(std::vector <double>& VecData, std::vector<double>& FilterData);

void fitLine(vector<POINT> &line,LINEKB & linekb);

#endif // ALGORITHM_H
