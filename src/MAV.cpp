#include "MAV.h"
#include "GL/glut.h"
#include "PelicanCtrl/gotoPos.h"
#include "tf/tf.h"
#include "HuskyInterface.h"

using namespace TooN;

double MAV::speed = 1;

MAV::MAV()
{
    pos = makeVector(0,0,10);
    speed = 1;
    realpos = makeVector(0,0,0,0);
    yaw = 0;
}

void MAV::Init(ros::NodeHandle *nh_, bool simulation_)
{
    nh = nh_;
    simulation = simulation_;

    if(!simulation)
    {
        gotoPosService = nh->serviceClient<PelicanCtrl::gotoPos>("/PelicanCtrl/gotoPos");
        atGoalSub = nh->subscribe("/PelicanCtrl/at_goal", 10, &MAV::atGoalCallback, this);
        gpsPose_sub = nh->subscribe("/PelicanCtrl/fixedPose", 10, &MAV::gpsPoseCallback, this);
        gps_sub = nh->subscribe("/fcu/gps", 10, &MAV::gpsCallback, this);
        //test();
    }
}

void MAV::test()
{
    SetGoal(makeVector(0,0,-5));
}

void MAV::gpsPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &msg)
{
    realpos[0] = msg->pose.pose.position.x;
    realpos[1] = msg->pose.pose.position.y;
    realpos[2] = msg->pose.pose.position.z;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    tf::Matrix3x3 rm;
    tf::Quaternion qtr(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    rm.setRotation(qtr);
    double r1,r2,r3;
    rm.getEulerYPR(r1,r2,r3);

    ROS_INFO_THROTTLE(3, "POS: %f %f %f rot: %f %f %f", realpos[0], realpos[1], realpos[2], r1, r2, r3);
    realpos[3] = 0;
}

void MAV::gpsCallback(const sensor_msgs::NavSatFixPtr &msg)
{
    //static bool sent=false;
    gpsLocation = (*msg);

//    if(!sent)
//    {
//        HuskyInterafce::Instance()->SendWaypoint(gpsLocation);
//        ROS_INFO("Sent GPS to husky.");
//        sent = true;
//    }

}

void MAV::atGoalCallback(const std_msgs::Bool::Ptr &msg)
{
    if(msg->data == true)
        atGoal = true;
}

void circle(float x, float y, float z, float r, int segments)
{
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(x, y, z);
    for( int n = 0; n <= segments; ++n )
    {
        float const t = 2*M_PI*(float)n/(float)segments;
        glVertex3f(x + sin(t)*r, y + cos(t)*r, z);
    }
    glEnd();
}

double r()
{
    return (-50.0+rand()%100)/5000.0;
}

void glVertex(Vector<3> p)
{
    glVertex3f(p[0],p[1],p[2]);
}

void MAV::glDraw()
{
    //ROS_INFO_THROTTLE(1, "AtGoal:%d", atGoal);
    Vector<3> p;
    if(simulation)
    {
        p = pos;
    }
    else
    {
        p[0] = realpos[0];p[1] = realpos[1];p[2] = realpos[2];
    }

    glColor3f(0,0,1);
    glLineWidth(2);
    double l=0.25;
    glBegin(GL_LINES);
    Vector<3> p1,p2,p3,p4;
    p1=p+makeVector(-l+r(),+r(),0);
    p2=p+makeVector(+l+r(),+r(),0);
    p3=p+makeVector(+r(),-l+r(),0);
    p4=p+makeVector(+r(),+l+r(),0);
    glVertex(p1);
    glVertex(p2);
    glVertex(p3);
    glVertex(p4);
    glEnd();

    glColor4f(0,0,0,0.5);
    circle(p1[0]+r(),p1[1]+r(),p1[2], 0.15, 50);
    circle(p2[0]+r(),p2[1]+r(),p2[2], 0.15, 50);
    circle(p3[0]+r(),p3[1]+r(),p3[2], 0.15, 50);
    circle(p4[0]+r(),p4[1]+r(),p4[2], 0.15, 50);

    glPointSize(10);
    glColor3f(0,0,1);
    glBegin(GL_POINTS);
    glVertex3f(goal[0], goal[1], goal[2]);
    glEnd();
}

void MAV::SetGoal(TooN::Vector<3> goalpos, bool set_orig)
{
    goal = goalpos;
    atGoal = false;

    if(simulation)
    {
        toGoalNorm = (goal-pos);
        normalize(toGoalNorm);
    }
    else
    {
        PelicanCtrl::gotoPos srv;
        srv.request.x = goal[0];
        srv.request.y = goal[1];
        srv.request.z = goal[2];
        srv.request.yaw = 0;
        srv.request.set_orig = set_orig;

        if(gotoPosService.call(srv))
        {
            ROS_INFO("Waypoint is sent to Pelican Controller");
        }
        else
        {
            ROS_INFO("** Unable to call gotoPos service on Pelican Controller");
        }
    }
}

void MAV::Update(double dt)
{
    if(simulation)
    {
        if(atGoal)
            return;

        double step = dt * speed;
        double goalDistSqr = (goal - pos)*(goal-pos);
        if(goalDistSqr < step*step || goalDistSqr < 0.2*0.2)
        {
            pos = goal;
            atGoal = true;

        }
        else
        {
            pos += step*toGoalNorm;
        }
    }
    else
    {
        //We don't need to check if MAV is at goal since
        //it will be set in upon receiving the AtGoal message
        //ROS_INFO_THROTTLE(1, "AtGoal:%d", atGoal);
    }
}
