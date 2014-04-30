#include "NUCParam.h"

double NUCParam::area_rotation = 0;
double NUCParam::cx = 0;
double NUCParam::cy = 0;
double NUCParam::sensingTime = 2;
double NUCParam::min_footprint = 4.0;

void NUCParam::GetParams(ros::NodeHandle &nh)
{
    nh.param<double>("area_rotation",area_rotation,0);
    nh.param<double>("sensing_time",sensingTime,2);
    nh.param<double>("min_footprint",min_footprint,2);
}
