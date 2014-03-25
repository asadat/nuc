#ifndef _NUC_
#define _NUC_

#include "ros/ros.h"

class NUC
{
public:

    ~NUC();

    static NUC * Instance(int argc=0, char **argv=NULL)
    {
        if(instance == NULL)
        {
            instance = new NUC(argc, argv);
        }

        return instance;
    }

    void idle();
   // void mainLoop();
//    void hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey);
    void glDraw();
//    void gpsPositionCallback(const asctec_hl_comm::PositionWithCovarianceStamped::Ptr &msg);
//    void gpsPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg);

    bool VisEnabled(){return bVisEnabled;}

private:

    NUC(int argc, char **argv);
    static NUC* instance;

    ros::NodeHandle nh;

//    ros::Subscriber gpsPos_sub;
//    ros::Subscriber gpsPose_sub;

    bool bVisEnabled;


};

#endif
