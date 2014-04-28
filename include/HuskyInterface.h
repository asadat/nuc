#ifndef _HUSKY_INTERFACE_
#define _HUSKY_INTERFACE_

#include "ros/ros.h"
#include "gps_waypoint_publisher/AddWaypoint.h"

class HuskyInterafce
{
public:
    static HuskyInterafce* Instance(ros::NodeHandle* nh_, bool simulation_)
    {
        if(instance == NULL)
        {
            instance = new HuskyInterafce(nh_, simulation_);
        }

        return instance;
    }

    void SendWaypoint(sensor_msgs::NavSatFix & wp);
private:
    HuskyInterafce(ros::NodeHandle* nh_, bool simulation_);
    static HuskyInterafce* instance;
    ros::ServiceClient addWPService;

};

#endif
