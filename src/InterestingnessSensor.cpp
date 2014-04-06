
#include "InterestingnessSensor.h"


InterestingnessSensor * InterestingnessSensor::instance;

InterestingnessSensor::InterestingnessSensor(ros::NodeHandle * nh_)
{
    nh = nh_;
    image_transport::ImageTransport it(*nh);
    img_sub = it.subscribe("camera/image_raw", 1, &InterestingnessSensor::imageCallback, this, image_transport::TransportHints("raw", ros::TransportHints().tcpNoDelay(true)));
}

void InterestingnessSensor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    imagePtr = cv_bridge::toCvCopy(msg);
}

