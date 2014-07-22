#ifndef _MAV_
#define _MAV_
#include "TooN/TooN.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "NUCParam.h"

class MAV
{
public:
    MAV();
    ~MAV(){}

    void Init(ros::NodeHandle* nh_, bool simulation_);
    void glDraw();
    void SetGoal(TooN::Vector<3> goalpos, bool set_orig=false);
    void Update(double dt);
    bool AtGoal(){return atGoal;}
    void atGoalCallback(const std_msgs::Bool::Ptr &msg);
    void gpsPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg);
    void gpsCallback(const sensor_msgs::NavSatFixPtr &msg);
    sensor_msgs::NavSatFix GetLastGPSLocation(){return gpsLocation;}

    void test();
    static void ChangeSpeed(double ds){NUCParam::speed +=ds;}
    //static double speed;
private:

    TooN::Vector<4,double> realpos; // onlu for visualization of real MAV
    TooN::Vector<3,double> pos;
    double yaw;
    TooN::Vector<3,double> goal;
    TooN::Vector<3,double> toGoalNorm;
    bool atGoal;
    bool simulation;

    ros::NodeHandle * nh;
    ros::ServiceClient gotoPosService;
    ros::Subscriber atGoalSub;
    ros::Subscriber gpsPose_sub;
    ros::Subscriber gps_sub;

    sensor_msgs::NavSatFix gpsLocation;

};

#endif
