//
// Created by robocon on 11/6/17.
//

#include "Radar.h"
#include "Base.h"

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
using namespace cv;
using namespace std;
namespace hitcrt
{
    // public
    Radar::Radar():serial("/dev/ttyUSB0",115200)
    {
        char * tmp[20];
        Connection_information information(0,tmp);
        // Connects to the sensor
        if (!urg.open(information.device_or_ip_name(),
                  information.baudrate_or_port_number(),
                  information.connection_type())) {
            cout << "Urg_driver::open(): "
                 << information.device_or_ip_name() << ": " << urg.what() << endl;
        }
    }
    Radar::~Radar()
    {
        if(urg.is_open())
            urg.close();
    }
    void Radar::init()
    {
        urg.set_scanning_parameter(urg.deg2step(-90), urg.deg2step(+90), 0);
        urg.start_measurement(Urg_driver::Distance, Urg_driver::Infinity_times, 0);
    }
    int Radar::run()
    {
        enum{Capture_times = 1};
        for(int i = 0;i<Capture_times;i++)
        {
            long time_stamp = 0;
            vector<long> data;
            if (!urg.get_distance(data, &time_stamp)) {
                cout << "Urg_driver::get_distance(): " << urg.what() << endl;
                return 1;
            }
            vector<POINT> VecPoint;
            depth2xy(data,VecPoint);
            myimshow(VecPoint);
            vector<vector<POINT>>lines;
            MyRansc(VecPoint,lines);
            cout <<"lines.size = "<<lines.size();
            getRadarPosition(lines);
        }
    }
// private
    void Radar::depth2xy(const vector<long> &data, vector<POINT> &VecPoint)
    {
        long min_distance = urg.min_distance();
        long max_distance = urg.max_distance();
        double xMax = 0;double yMax = 0;
        size_t size = data.size();
        double radian;
        for(int i=0;i<size;i++)
        {
            long l = data[i];
            if((l<=min_distance)||(l>=max_distance))
                continue;
            radian = urg.index2rad(i);
            POINT tmp_point;
            tmp_point.x = l*sin(radian);
            tmp_point.y = l*cos(radian);
            if(tmp_point.x>xMax) xMax = tmp_point.x;
            if(tmp_point.y>yMax) yMax = tmp_point.y;
            //cout <<i<<". "<<"theta = "<<radian<<" r = "<<l<<" x= "<<tmp_point.x<<" y= "<<tmp_point.y<<endl;
            //cout <<"theta = "<<radian<<" distance = "<<l<<endl;
            //
            //cout<<"x = "<<tmp_point.x<<" y = "<<tmp_point.y<<endl;
            VecPoint.push_back(tmp_point);
        }
        maxDisInXY = std::max(yMax,xMax);
    }
    bool Radar::sortLines(vector<int> & l1,vector<int> & l2)
    {
        return l1.front()<l2.front();
    }
    void Radar::MyRansc(vector<POINT> & VecPoint,vector<vector<POINT>> &lines)
    {
        const int MAXLINENUM = 5;
        const int ITERANUM = 100;
        const double disDelta = 30;
        const int MINLINELEN = 30;
        const int MAXNOISENUM = 30;
        vector<int> maxLineIndex;           //ransc找到最优模型的索引
        vector<vector<int>> LINES;        //记录找到的线段的索引
//        vector<POINT> L;        //调试观测分离直线用
        vector<int> VecIndex(VecPoint.size(),0);     //记录点是否被分离出
        vector<int> leftPointsIndex;      //记录被分离后剩余点的索引
        vector<int> linePointIndex;        //记录每次迭代暂存符合点
        vector<POINT>findLine;
        const vector<Scalar> color ={Scalar(255,80,80),Scalar(80,255,80),Scalar(80,80,255)};

        for(int i=0;i<MAXLINENUM;i++)    //最多抽取5条直线
        {
            for(int n=0;n<VecIndex.size();n++)  //找出点集中分离直线点后的结果
                if(VecIndex[n]==0)
                    leftPointsIndex.push_back(n);
            cout <<"leftpoints.size = "<<leftPointsIndex.size()<<" VecSize = "<<VecIndex.size()<<endl;
            if(leftPointsIndex.size()<MAXNOISENUM)   //提取直线点后剩余点小于阈值认为剩余为噪声
            {
                cout <<"isnoise"<<endl;
                break;
            }
            for (int k = 0; k <ITERANUM ; ++k)   //迭代取出当前点集中最长线段
            {
                LINE L1={RANDOMPOINT,RANDOMPOINT};          //当前点集中取两个随机点确定一条直线
                for(auto j:leftPointsIndex)
                {
                    double dis = CalPointLineDistance(VecPoint[j],L1);
                    if(dis <disDelta)
                    {
                        linePointIndex.push_back(j);
                    }
                }
                if(linePointIndex.size()>maxLineIndex.size())
                {
                    maxLineIndex = linePointIndex;
                }
                linePointIndex.clear();
            }
            if (maxLineIndex.size()< MINLINELEN)                             //从点集中抽离出找到的线段
                break;
            cout <<"maxline.size() = "<<maxLineIndex.size()<<endl;
            LINES.push_back(maxLineIndex);
            for(auto index:maxLineIndex)
            {
//                L.push_back(VecPoint[index]);
                VecIndex.at(index) = 1;
            }
//            myimshow(L);
//            L.clear();
            maxLineIndex.clear();
            leftPointsIndex.clear();
        }
        cout <<"LINES.size()= "<< LINES.size()<<endl;
        //for(auto l:LINES)                           //把所有的直线点合在一起
            //resultIndex.insert(resultIndex.end(),l.begin(),l.end());
        sort(LINES.begin(),LINES.end());        //按激光雷达扫描顺序给找到点排序
        Mat image = Mat::zeros(800,1200,CV_8UC3);
        Mat fitlineimage = Mat::zeros(800,1200,CV_8UC3);
        LINEKB kb;
        for(int i=0;i<LINES.size();i++)
        {
            for(auto pIndex:LINES[i])
                findLine.push_back(VecPoint[pIndex]);
            cout <<"l.size = "<<LINES[i].size()<<" l.front = "<<LINES[i].front()<<endl;
            findLine=KalmanFiltePoints(findLine);
            lines.push_back(findLine);
            myimshow(findLine);
            fillMat(image,findLine,i<3?color[i]:RANDCOLOR);
            // fill the line num
            stringstream text;
            text<<i;
            POINT textPos = world2pixel(findLine.front());  //num
            putText(image,text.str(),Point(textPos.x,textPos.y),FONT_HERSHEY_PLAIN,1,Scalar(255,0,0),2);
            fitLine(findLine,kb);
            POINT p1 = findLine.front();
            POINT p2 = findLine.back();
            p1.y = kb.k*p1.x+kb.b;
            p2.y = kb.k*p2.x+kb.b;
            p1 = world2pixel(p1);p2 = world2pixel(p2);
            line(fitlineimage,Point(p1.x,p1.y),Point(p2.x,p2.y),color[i]);
            circle(fitlineimage,Point(p1.x,p1.y),4,Scalar(255,255,0),-1);
            circle(fitlineimage,Point(p2.x,p2.y),4,Scalar(255,255,0),-1);
            findLine.clear();
        }
        imshow("radarImage",image);
        imshow("fitlineImage",fitlineimage);
        waitKey();

    }

    void Radar::myimshow(vector<POINT> points)
    {
        Mat image = Mat::zeros(800,1200,CV_8UC3);
        POINT p;
        for(auto point:points)
        {
            p = world2pixel(point);
            if(p.y>=0&&p.y<=800&&p.x>=0&&p.x<=1200)
                image.at<Vec3b>(p.y,p.x)[1] = 188;
            else
                cout <<"p.x = "<<p.x<<" p.y = "<<p.y<<endl;
        }
        imshow("img",image);
        waitKey();
    }
    POINT Radar::world2pixel(POINT old)
    {
//        int divNum = (int)maxDisInXY/610;
        int divNum = 14;
        POINT trans;
        trans.x = (int)old.x/divNum+600;
        trans.y = 800-(int)old.y/divNum;
        return trans;
    }
    void Radar::fillMat(Mat &img, vector<POINT> &points, Scalar color)
    {
        for(auto p:points)
        {
            p = world2pixel(p);
            if(p.y>=0&&p.y<=800&&p.x>=0&&p.x<=1200)
            {
                img.at<Vec3b>(p.y,p.x)[2] = color[0];
                img.at<Vec3b>(p.y,p.x)[1] = color[1];
                img.at<Vec3b>(p.y,p.x)[0] = color[2];
            }
            else
                cout <<"p.x = "<<p.x<<" p.y = "<<p.y<<endl;
        }
    }
    void Radar::getRadarPosition(vector<vector<POINT>>&lines)
    {
        cout<<"getRadarPosition:  lines.size = "<<lines.size()<<endl;
        if(lines.size()>2)
        {
            LINE boundary = {lines.back()[5],lines.back()[lines.back().size()-5]};
            POINT center={0,0};
            double x = CalPointLineDistance(center,boundary);
            vector<float>data;
            data.push_back(static_cast<float>(x));
            cout <<"radar.x = "<<x<<endl;
            serial.send(SerialApp::SEND_RADAR,data);
            cout <<"send ok"<<endl;
        }
        else
            cout<<"lines find failed"<<endl;

    }
    vector<POINT> Radar::KalmanFiltePoints(const vector<POINT> VecPoint)
    {
        vector<POINT> FilterPoint;
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
        return FilterPoint;
    }
}
