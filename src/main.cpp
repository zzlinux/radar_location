#include <iostream>
#include "Base.h"
#include "serialbase.h"
#include "serialapp.h"
using namespace hitcrt;
int main() {

    std::cout << "Hello, World!" << std::endl;
    Base base;
    base.init();
    base.run();
//    SerialApp serial("/dev/ttyUSB0",115200);
//    unsigned char data[10]={1,2,3,4,5,6,7,8,9,10};
//    unsigned char re[10];
//    vector<float> a ={1,2,3,4};
//    serial.send(SerialApp::SEND_RADAR,a);
//    SerialApp::RECEIVE_FLAG flag;
//    vector<float> b;
//    serial.receive(flag,b);
//    for(auto i:b)
//        cout <<i<<endl;
    return 0;
}