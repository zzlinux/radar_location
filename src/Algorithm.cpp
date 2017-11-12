#include "Algorithm.h"

using namespace std;


/*
 * @brief
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = 1;
 *     H = 1;
 *   and @q,@r are valued after prior tests.
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs
 *   state - Klaman filter structure
 *   init_x - initial x state value
 *   init_p - initial estimated error convariance
 * @outputs
 * @retval
 */
void kalman1_init(kalman1_state *state, float init_x, float init_p)
{
    state->x = init_x;
    state->p = init_p;
    state->A = 1;
    state->H = 1;
    state->q = 0.008;//10e-6  0.008;  /* predict noise convariance */
    state->r = 0.7;//10e-5;      0.7/* measure error convariance */
}

/*
 * @brief
 *   1 Dimension Kalman filter
 * @inputs
 *   state - Klaman filter structure
 *   z_measure - Measure value
 * @outputs
 * @retval
 *   Estimated result
 */
float kalman1_filter(kalman1_state *state, float z_measure)
{
    /* Predict */
    state->x = state->A * state->x;
    state->p = state->A * state->A * state->p + state->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Measurement */
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
    state->x = state->x + state->gain * (z_measure - state->H * state->x);
    state->p = (1 - state->gain * state->H) * state->p;

    return state->x;
}

/****************************************************/
/**基于曲率的特征分割算法**/
/**参考王永锟学长论文**/
/****************************************************/
//计算两点之间距离
double CalDistance(const POINT &point1, const POINT &point2)
{
    return sqrt((point1.x-point2.x)*(point1.x-point2.x) + (point1.y-point2.y)*(point1.y-point2.y));
}
//计算线段长度
double CalLineLength(LINE &line)
{
    return sqrt((line.PointStart.x-line.PointEnd.x)*(line.PointStart.x-line.PointEnd.x)
                    + (line.PointStart.y-line.PointEnd.y)*(line.PointStart.y-line.PointEnd.y) );
}
//计算点到直线之间距离
double CalPointLineDistance(const POINT &point, const LINE &line)
{
    //计算直线方程
    if(line.PointStart.x == line.PointEnd.x)//直线斜率无穷大
        return abs(point.x - line.PointStart.x);
    else
    {
        double line_k, line_b;

        line_k = (line.PointStart.y - line.PointEnd.y)/(line.PointStart.x - line.PointEnd.x);
        line_b = line.PointStart.y - line_k*line.PointStart.x;
        return abs(line_k*point.x - point.y + line_b)/sqrt(line_k*line_k + 1);
    }
}

//求一个vector的中间点与首尾点构成的曲线曲率
double CalCurvature(const std::vector<POINT> &VecPoint)
{
    size_t i = VecPoint.size();
    POINT point;
    LINE line;

    line.PointStart = VecPoint.front();
    line.PointEnd = VecPoint.back();
    point = VecPoint.at((i+1)/2);

    return CalPointLineDistance(point, line)/ CalLineLength(line);
}

//返回一个vector的中间点的下标值
size_t GetMidIndexofVector(const vector<POINT> &VecPoint)
{
    return (VecPoint.size() + 1)/2;
}

//以11个点为周期，循环计算一个vector中各点的曲率
void CalVecCurvature(const vector<POINT> &VecPoint, vector<double>&PointCurvature)
{
    const size_t vec_unit = 10;
    const size_t vec_size = VecPoint.size();
    vector<POINT> point;

    for(size_t i=0; i<vec_size-vec_unit; i++)//11点循环赋值
    {
        point.assign(VecPoint.begin()+i, VecPoint.begin()+i+vec_unit);
        double tmp_curv = CalCurvature(point);
        PointCurvature.push_back(tmp_curv);//曲率计算结果赋值 一共vec_size-vec_unit个值
        //cout<< i<<"    "<<tmp_curv<<endl;
    }
}

//求角点下标：该角点曲率>Kcor且比它后面的20个点的曲率都大
void CalPeakValIndex(vector<double>&PointCurvature, vector<size_t>& PeakIndex)
{
    const double Kcor = 0.08f;       //角点阈值曲率，实验确定
    const size_t   PeakIsTrue = 20; //20个数据范围判断

    size_t    PeakFlag = 0;
    bool      IsCorner = false;
    double CurvMax = Kcor;
    size_t    CurvMaxIndex = 0;

    for(size_t i=0; i<PointCurvature.size();i++)
    {
        double tmp_cur = PointCurvature.at(i) ;
        if(tmp_cur > Kcor && tmp_cur > CurvMax)//大于阈值曲率 且 大于上一个CurvMax
        {
            CurvMax = tmp_cur;
            CurvMaxIndex = i;
            PeakFlag = 0;
            IsCorner = true;
        }
        else
        {
            if(PeakFlag == PeakIsTrue && IsCorner == true)//峰值有效
            {
                PeakIndex.push_back(CurvMaxIndex);//存储峰值下标
      //          cout<< "PeakVal =    "<<CurvMax<<endl;//曲率峰值打印
      //          cout<< "PeakIndex = "<<CurvMaxIndex<<endl;//曲率下标打印

                CurvMax = Kcor;//CurMax复位,便于判断下一个峰值
                IsCorner = false;
            }
        }
        PeakFlag++;
    }
}

//由索引下标计算激光雷达到拟合的直线距离
void CalRadarCordinates(vector<POINT>  &VecPoint, vector<size_t> &CornerIndex, RADAR_CORDINATES &RadarCordinates)
{
    size_t CornerNum = CornerIndex.size();//角点个数
    LINE line;
    POINT point = {0,0};
    double x_distance;//距离缓存
    double y_distance;

    //坐标都是按朝向顺时针角点计算
    if(CornerNum == 1)//只有一个角点,两段直线
    {
        line.PointStart = VecPoint.front();
        line.PointEnd = VecPoint.at(CornerIndex.front());
        x_distance = CalPointLineDistance(point, line);//x坐标

        line.PointStart = VecPoint.back();
        y_distance = CalPointLineDistance(point, line);//y坐标
    }
    else if(CornerNum == 2)//两个角点，三段直线 坐标自纠正
    {
        if(CornerIndex.front() > 80)//第一段直线多于80个点则采用前两段直线计算
        {
            line.PointStart = VecPoint.front();
            line.PointEnd = VecPoint.at(CornerIndex.front());
            x_distance = CalPointLineDistance(point, line);//x坐标

            line.PointStart = VecPoint.at(CornerIndex.back());
            y_distance = CalPointLineDistance(point, line);//y坐标
        }
        else//否则采用后两段直线计算
        {
            line.PointStart = VecPoint.at(CornerIndex.front());
            line.PointEnd = VecPoint.at(CornerIndex.back());
            y_distance = CalPointLineDistance(point, line);//x坐标

            line.PointStart = VecPoint.back();
            x_distance =  BASE_X_LENGTH - CalPointLineDistance(point, line);//y坐标
        }
    }
    else if(CornerNum == 3)//三个角点四段直线，采用中间两段直线计算
    {
        line.PointStart = VecPoint.at(CornerIndex.at(0));
        line.PointEnd = VecPoint.at(CornerIndex.at(1));
        x_distance = CalPointLineDistance(point, line);//x坐标

        line.PointStart = VecPoint.at(CornerIndex.at(2));
        y_distance = CalPointLineDistance(point, line);//y坐标
    }
    else
    {
        cout<<"Detected CornerNum is "<<CornerNum<<endl;
        return;//其他情况跳出本次运算
    }

    //求激光雷达中心坐标
    RadarCordinates.x = x_distance;
    RadarCordinates.y = y_distance;
}

//计算姿态角，定义首位两端直线与Y轴负方向夹角为正，范围：45---90---135
/*捕捉到一个角点时，Corner_Rignt 范围：(45, 90)
**                                  Corner_Left   范围：(90, 135)
**捕捉到两个角点时，无需角度计算辅助
**捕捉到三个角点时，情况同一个角点相同
*/
double CalRobotAngle(vector<POINT>  &VecPoint)
{
    //检查错误
    size_t vec_size = VecPoint.size();
    if(vec_size < 400)
    {
        cout<<"VecPoint's size is less than 400"<<endl;
        return 0.0f;
    }

    //分别取前后5个点的平均值作为始末点
    POINT point1;
    POINT point2;

    double sumx = 0;
    double sumy = 0;
    for(size_t i=0; i<5; i++)
    {
        sumx += VecPoint.at(i).x;
        sumy += VecPoint.at(i).y;
    }
    point1.x = sumx/5;
    point1.y = sumy/5;

    sumx = 0;
    sumy = 0;
    for(size_t i=1; i<6; i++)
    {
        sumx += VecPoint.at(vec_size-i).x;
        sumy += VecPoint.at(vec_size-i).y;
    }
    point2.x = sumx/5;
    point2.y = sumy/5;

    //求首位点构成的向量
    POINT stVector1;
    stVector1.x = point2.x - point1.x;
    stVector1.y = point2.y - point1.y;

    POINT stVector2 = {0, 1};//单位向量

    //求两向量夹角
    return RAD_DEG * acos((stVector1.x*stVector2.x + stVector1.y*stVector2.y)
                / sqrt((stVector1.x*stVector1.x + stVector1.y*stVector1.y)  *  (stVector2.x*stVector2.x + stVector2.y*stVector2.y))   );
}

//递推平均滤波算法
#define N 10
void GlideFilter(vector <double>& VecData, vector<double>& FilterData)
{
    size_t n = VecData.size();
    size_t Filter_i = 0;

    double Sum = 0.0f;
    double  DataBuf[N] = {0};

    for(size_t i=0; i<n;i++)
    {
        DataBuf[Filter_i++] = VecData.at(i);
        if(Filter_i==N)
                Filter_i = 0;
        for(size_t j=0;j<N;j++)
            Sum += DataBuf[j];
        FilterData.push_back(Sum/N);
        Sum = 0;
    }
}

// 求雷达方向
void CalRadarDirection(LINE & liney,double & direction)
{
    POINT vecLiney={liney.PointEnd.x-liney.PointStart.x,liney.PointEnd.y-liney.PointStart.y};
    double ab = vecLiney.y;
    direction = acos(ab/sqrt(vecLiney.y*vecLiney.y+vecLiney.x*vecLiney.x));
}

void fitLine(vector<POINT> &line,LINEKB & linekb)
{
    double A=0,B=0,C=0,D=0;
    for(auto point:line)
    {
        A+=point.x*point.x;
        B+=point.x;
        C+=point.x*point.y;
        D+=point.y;
    }
    double div = line.size()*A-B*B;
    if(div != 0)
    {
        linekb.k = (line.size()*C-B*D)/div;
        linekb.b = (A*D-B*C)/div;
    }
    else
    {
        linekb.k = 1;
        linekb.b = 0;
    }
}
