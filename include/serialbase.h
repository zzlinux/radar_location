#ifndef HITCRT_SERIAL_BASE_H_
#define HITCRT_SERIAL_BASE_H_

#include <boost/asio.hpp>

namespace hitcrt
{

class SerialBase
{
public:
    SerialBase(std::string str, int baud_rate);
    ~SerialBase();
    void send(unsigned char* ch, size_t length);
    void receive(unsigned char* buff, size_t& length);
private:
    boost::asio::io_service* io;
    boost::asio::serial_port* port;

    static const int MAX_BUFFER_LENGTH = 100;
};


}

#endif // HITCRT_SERIAL_BASE_H_
