#ifndef _MAV_
#define _MAV_
#include "TooN/TooN.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "asctec_hl_comm/mav_ctrl.h"
#include "NUCParam.h"

class MAV
{
public:
    MAV();
    ~MAV(){}

    class AsctecFCU
    {
        public:
            AsctecFCU();

            void Init(ros::NodeHandle* nh_);
            void fcuCtrlCallback(const asctec_hl_comm::mav_ctrl::Ptr &msg);
            void Update();
            void SetPose(TooN::Vector<3>);
            TooN::Vector<4> vel;
            double delay;
        private:
            ros::Publisher fcuPose_pub;
            ros::Publisher fcuMag_pub;
            ros::Subscriber fcuCtrl_sub;

            TooN::Vector<4> pose;
            std::vector<std::pair<double,TooN::Vector<4> > > poseQ;

    };

    void Init(ros::NodeHandle* nh_, bool simulation_);
    void glDraw();
    void SetGoal(TooN::Vector<3> goalpos, bool set_orig=false);
    void Update(double dt);
    void JumpToGoal();
    bool AtGoal(){return atGoal;}
    void atGoalCallback(const std_msgs::Bool::Ptr &msg);
    void gpsPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg);
    void gpsCallback(const sensor_msgs::NavSatFixPtr &msg);
    sensor_msgs::NavSatFix GetLastGPSLocation(){return gpsLocation;}

    void test();
    static void ChangeSpeed(double ds){NUCParam::speed +=ds;}
    TooN::Vector<3> GetPos(){return realpos;}
    //static double speed;
private:

    TooN::Vector<3,double> realpos; // onlu for visualization of real MAV
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
    AsctecFCU asctecPelican;
};

#endif
