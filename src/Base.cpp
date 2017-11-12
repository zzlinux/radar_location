//
// Created by robocon on 11/6/17.
//

#include "Base.h"
namespace hitcrt
{
    Base::Base()
    {
//        serial = std::unique_ptr<SerialApp> (new SerialApp("/dev/ttyUSB0",115200));
        radar = std::unique_ptr<Radar>(new Radar);
    }
    void Base::init()
    {
        radar->init();
    }
    void Base::run()
    {
        radar->run();
    }
}