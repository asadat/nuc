
#include "NUC.h"
#include "GL/glut.h"


NUC * NUC::instance = NULL;

NUC::NUC(int argc, char **argv)
{

    bVisEnabled = true;

//    gpsPos_sub = nh.subscribe("/fcu/gps_position", 100, &NUC::gpsPositionCallback, this);
//    gpsPose_sub = nh.subscribe("/fcu/gps_pose", 100, &NUC::gpsPoseCallback, this);

    for(int i=1; i<argc;i++)
    {
        if(strcmp(argv[i],"-novis")==0)
        {
            bVisEnabled = false;
        }
    }

    glutInit(&argc, argv);
}

NUC::~NUC()
{

}
//void NUC::gpsPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg)
//{
//    Vector<3> p;
//    p[0] = msg->pose.pose.position.x;
//    p[1] = msg->pose.pose.position.y;
//    p[2] = msg->pose.pose.position.z;

//    p_pos.push_back(p);

//    Vector<4> att;
//    att[0] = msg->pose.pose.orientation.x;
//    att[1] = msg->pose.pose.orientation.y;
//    att[2] = msg->pose.pose.orientation.z;
//    att[3] = msg->pose.pose.orientation.w;

//    p_att.push_back(att);

//}

//void NUC::gpsPositionCallback(const asctec_hl_comm::PositionWithCovarianceStamped::Ptr &msg)
//{
//    Vector<3> p;
//    p[0] = msg->position.x;
//    p[1] = msg->position.y;
//    p[2] = msg->position.z;

//    positions.push_back(p);

//}

void NUC::glDraw()
{
     float w=10;
     glLineWidth(2);
     glColor3f(0,0,0);
     glBegin(GL_LINES);
     for(int i=0; i<=w; i++)
     {
     glVertex3f(-w/2, -w/2+i, 0);
     glVertex3f( w/2, -w/2+i, 0);
     }

     for(int i=0; i<=w; i++)
     {
     glVertex3f(-w/2+i, -w/2, 0);
     glVertex3f(-w/2+i, w/2, 0);
     }

     glEnd();



}

//void NUC::hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey)
//{
//    updateKey = true;

////    if(key['`'])
////    {
////        world->PopulateWorld(20);
////    }

//}

void NUC::idle()
{
    static double rosFreq=15;
    static double ros_p = 1/rosFreq;
    static ros::Time lastTime = ros::Time::now();
    //static int i=0;

    if((ros::Time::now()-lastTime).toSec() > ros_p)
    {
        lastTime = ros::Time::now();
        if(ros::ok())
        {
           //ROS_INFO_THROTTLE(0.5, "Ros spinning ... %d", i);
            ros::spinOnce();
           // i++;
        }
        else
        {
            exit(0);
        }
    }

}

