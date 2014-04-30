#ifndef _NUCPARAM_
#define _NUCPARAM_

#include "ros/ros.h"

#define D2R 3.14/180

class NUCParam
{
public:
    static void GetParams(ros::NodeHandle &nh);
    static double area_rotation;
    static double cx;
    static double cy;
    static double sensingTime;
    static double min_footprint;
};

#endif
