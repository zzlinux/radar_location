#include "serialbase.h"


namespace hitcrt
{

SerialBase::SerialBase(std::string str, int baud_rate)
{
    io = new boost::asio::io_service;
    port = new boost::asio::serial_port(*io, str.c_str());
    port->set_option(boost::asio::serial_port::baud_rate(baud_rate));
    port->set_option(boost::asio::serial_port::flow_control());
    port->set_option(boost::asio::serial_port::parity());
    port->set_option(boost::asio::serial_port::stop_bits());
    port->set_option(boost::asio::serial_port::character_size(8));
}

SerialBase::~SerialBase()
{
    delete io;
    delete port;
}

void SerialBase::send(unsigned char* ch, size_t length)
{
    boost::asio::write(*port, boost::asio::buffer(ch, length));
}


void SerialBase::receive(unsigned char* buff, size_t& length)
{
    boost::system::error_code err;
    length = port->read_some(boost::asio::buffer(buff, 100), err);
}




}
