#include "ros/ros.h"
#include "asctec_hl_comm/PositionWithCovarianceStamped.h"
#include "TooN/TooN.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
class PositionVis
{
public:

    ~PositionVis();

    static PositionVis * Instance(int argc=0, char **argv=NULL)
    {
        if(instance == NULL)
        {
            instance = new PositionVis(argc, argv);
        }

        return instance;
    }

    void idle();
    void mainLoop();
    void hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey);
    void glDraw();
    void gpsPositionCallback(const asctec_hl_comm::PositionWithCovarianceStamped::Ptr &msg);
    void gpsPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg);

private:

    PositionVis(int argc, char **argv);
    static PositionVis* instance;

    ros::NodeHandle nh;
    ros::Subscriber gpsPos_sub;
    ros::Subscriber gpsPose_sub;

    std::vector<TooN::Vector<3> > positions;
    std::vector<TooN::Vector<3> > p_pos;
    std::vector<TooN::Vector<4> > p_att;

};
