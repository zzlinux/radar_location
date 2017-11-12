//
// Created by robocon on 11/6/17.
//

#ifndef RADAR_LOCATION_BASE_H
#define RADAR_LOCATION_BASE_H

#include <opencv2/opencv.hpp>
#include "serialapp.h"
#include "Radar.h"
namespace hitcrt {
class Base {
private:
//    std::unique_ptr<SerialApp> serial;
    std::unique_ptr<Radar> radar;
    int a;
public:
    Base();
    ~Base(){};
    void init();
    void run();
};
}

#endif //RADAR_LOCATION_BASE_H
