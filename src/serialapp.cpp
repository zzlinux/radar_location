/********************************************************

串口协议为 55 00 ID NUM NUM*float 00 AA

比如 功能ID为 03
发送数据为1个float 则协议为
55 00 03 01 xx xx xx xx 00 AA

该程序应用了boost asio 库



*********************************************************/



#include "serialapp.h"


namespace hitcrt
{

SerialApp::SerialApp(const std::string str, int baudrate)
{
    m_serialBase = new SerialBase(str.c_str(), baudrate);       ///串口基层，负责收发具体数据
    MAX_RECEIVE_FLOAT_LENGTH = 20;
    FIRST_ONE = 0x55;
    FIRST_TWO = 0x00;

    LAST_ONE = 0x00;  ///倒数第二
    LAST_TWO = 0xAA;  ///倒数第一

}

SerialApp::~SerialApp()
{
    delete m_serialBase;
}

void SerialApp::send(SEND_FLAG send_flag, std::vector<float> num)
{

    m_send_mutex.lock();

    int send_length = num.size()*4+6;
    assert(send_length < 100);
    unsigned char buff[100] = {};

    buff[0] = FIRST_ONE;
    buff[1] = FIRST_TWO;

    if(SEND_HEART_BEAT == send_flag)
        buff[2] = 0x01;
    if(SEND_ERR_U == send_flag)
        buff[2] = 0x02;
    if(SEND_RUNE == send_flag)
        buff[2] = 0x03;
    if(SEND_ERR_UV == send_flag)
        buff[2] = 0x04;
    if(SEND_RADAR == send_flag)
        buff[2] = 0x05;
    if(SEND_SUPPLY == send_flag)
        buff[2] = 0x06;
//
//
    buff[3] = static_cast<unsigned char>(num.size());

//    std::cout << "1" << std::endl;

    for(int i = 0;i < num.size();i++)
    {
        Float2uchar temp;
        temp.fl = num[i];       //why
        for(int j = 0;j < 4;j++)
            buff[i*4+4+j] = temp.ch[j];
    }

    buff[num.size()*4+4] = LAST_ONE;
    buff[num.size()*4+4+1] = LAST_TWO;

    m_serialBase->send(buff, send_length);

    m_send_mutex.unlock();
}
void SerialApp::receive(SerialApp::RECEIVE_FLAG& receive_flag, std::vector<float>& num)
{
    while(1)
    {
        unsigned char buff[100];
        size_t received_length;
        m_serialBase->receive(buff, received_length);

        if(decode(buff, received_length, num, receive_flag))
        {
//            cout << "receive_flag = " << receive_flag
            break;
        }
    }

}

bool SerialApp::decode(unsigned char* buff, size_t& received_length, std::vector<float>& num, SerialApp::RECEIVE_FLAG& receive_flag)
{
    static int flag = 1;        ///接收状态机
    static RECEIVE_FLAG temp_receive_flag;  ///接收标志量
    static std::vector<float> vec_num;     ///接收数据
    static int temp_num_length;
    static bool return_flag;

    return_flag = false;

    for(size_t i = 0 ;i < received_length;i++)
    {
        switch (flag)
        {
            case 1:     ///报头1
                vec_num.clear();
                if(FIRST_ONE == buff[i])
                    flag = 2;
                else
                    flag = 1;
                break;
            case 2:     ///报头2
                if(FIRST_TWO == buff[i])
                    flag = 3;
                else
                    flag = 1;
                break;
            case 3:     ///类型
                if(0x01 == buff[i])
                {
                    temp_receive_flag = RECEIVE_HEART_BEAT;
                    flag = 4;
                }
                else if(0x02 == buff[i])
                {
                    temp_receive_flag = RECEIVE_SHUT_DOWN;
                    flag = 4;
                }
                else
                    flag = 1;
                break;
            case 4:     ///浮点数长度
                temp_num_length = static_cast<int>(buff[i]);
                if(temp_num_length < 0 && temp_num_length > MAX_RECEIVE_FLOAT_LENGTH)
                    flag = 1;
                else if(temp_num_length == 0)
                    flag = 6;
                else
                    flag = 5;
                break;
            case 5:
                float temp;
                if(decode_num(buff[i], temp))
                    vec_num.push_back(temp);
                if(vec_num.size() == temp_num_length)
                    flag = 6;
                break;
            case 6:
                if(LAST_ONE == buff[i])
                    flag = 7;
                else
                    flag = 1;
                break;
            case 7:
                if(LAST_TWO == buff[i])
                {
                    receive_flag = temp_receive_flag;
                    num = vec_num;
                    return_flag = true;
                }
                flag = 1;
                break;
        }
    }
    return return_flag;

}

bool SerialApp::decode_num(unsigned char buff, float& num)
{
    static Float2uchar temp_num;
    static int pos = 0;
    temp_num.ch[pos] = buff;
    pos++;
    if(pos == 4)
    {
        num = temp_num.fl;
        pos = 0;
        return true;
    }
    return false;
}



}
