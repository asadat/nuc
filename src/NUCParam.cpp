#include "NUCParam.h"

double NUCParam::area_rotation = 0;
double NUCParam::cx = 0;
double NUCParam::cy = 0;
double NUCParam::sensingTime = 2;
double NUCParam::min_footprint = 4.0;
std::string NUCParam::log_folder = std::string("");
bool NUCParam::logging = false;
double NUCParam::FOV = 3.14/2.0;

void NUCParam::GetParams(ros::NodeHandle &nh)
{
    nh.param<double>("FOV",FOV,1.57);
    nh.param<double>("area_rotation",area_rotation,0);
    nh.param<double>("sensing_time",sensingTime,2);
    nh.param<double>("min_footprint",min_footprint,2);
    nh.param("logging", logging, false);
    nh.param("log_folder", log_folder, std::string(""));
}
