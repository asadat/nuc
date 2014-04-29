#include "NUCParam.h"

double NUCParam::area_rotation = 0;
double NUCParam::cx = 0;
double NUCParam::cy = 0;
double NUCParam::sensingTime = 2;

void NUCParam::GetParams(ros::NodeHandle &nh)
{
    nh.param<double>("area_rotation",area_rotation,0);
    nh.param<double>("sensing_time",sensingTime,2);
}
