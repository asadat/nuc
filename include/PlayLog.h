#include "ros/ros.h"
#include "asctec_hl_comm/PositionWithCovarianceStamped.h"
#include "asctec_hl_comm/mav_ctrl.h"
#include "TooN/TooN.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "GL/glut.h"

class PlayLog
{
public:

    ~PlayLog();

    static PlayLog * Instance(int argc=0, char **argv=NULL)
    {
        if(instance == NULL)
        {
            instance = new PlayLog(argc, argv);
        }

        return instance;
    }

    void idle();
    void mainLoop();
    void hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey);
    void glDraw();
    //void gpsPositionCallback(const asctec_hl_comm::PositionWithCovarianceStamped::Ptr &msg);
    void gpsPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg);
    void fixedPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg);
    void imuCallback(const sensor_msgs::Imu::Ptr &msg);

    void velCallback(const asctec_hl_comm::mav_ctrl::Ptr &msg);

private:

    PlayLog(int argc, char **argv);
    static PlayLog* instance;

    void Clear();

    ros::NodeHandle nh;

    ros::Subscriber imu_sub;
    ros::Subscriber fixedPose_sub;
    ros::Subscriber gpsPose_sub;
    ros::Publisher  position_pub;


    std::vector<TooN::Vector<3> > positions;
    std::vector<TooN::Vector<3> > p_pos;
    std::vector<TooN::Vector<4> > p_att;
    TooN::Vector<4> orig;
    TooN::Vector<3> p_orig;
    std::vector<double> rpy[3];
    TooN::Vector<4> curAtt;
    
    double fixedYaw;
    TooN::Vector<3> vel;

    std::vector<TooN::Vector<3> > waypoints;
    std::vector<TooN::Vector<4> > footprints;
    std::vector<std::string> rawTexFiles;
    std::vector<GLuint> gluintsRaw;
    std::vector<std::string> sensedTexFiles;

    bool drawImages;
    std::vector<GLuint> gluintsSensed;

    bool drawSensedImages;
    bool drawTrajectory;
    bool drawWaypoints;


};
