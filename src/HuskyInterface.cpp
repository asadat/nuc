#include "HuskyInterface.h"

HuskyInterafce* HuskyInterafce::instance = NULL;

HuskyInterafce::HuskyInterafce(ros::NodeHandle *nh_)
{
    addWPService = nh_->serviceClient<gps_waypoint_publisher::AddWaypoint>("/add_waypoint");
    
}

void HuskyInterafce::SendWaypoint(sensor_msgs::NavSatFix &wp)
{
    //gps.publish(wp);
    //return;

    gps_waypoint_publisher::AddWaypoint srv;
    srv.request.fix = wp;

    if(addWPService.exists())
    {
        addWPService.call(srv);
    }
    else
    {
        ROS_ERROR("AddWaypoint service is not available!");
    }

}
