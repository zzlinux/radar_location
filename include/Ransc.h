#ifndef RANSC_H
#define RANSC_H
#include "Algorithm.h"

void RANSC(const std::vector<POINT> P, size_t nrIterations, std::vector<POINT> & best_inliners);
//void KalmanFiltePoints(const std::vector<POINT> VecPoint, std::vector<POINT>  &FilterPoint);
void RadarLocation(const std::vector<POINT>VecPoint, RADAR_INFO &stRadarInfo);
void WritePointsToFile(const std::vector<POINT> FilterPoint, const char*  filename);
void WriteDataToFile(const std::vector<double>FilterPoint, const char*  filename);
void ReadPointsFromFile(std::vector<POINT> &FilterPoint, const char*  filename);


#endif // RANSC_H
