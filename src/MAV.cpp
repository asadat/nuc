#include "MAV.h"
#include "GL/glut.h"
#include "PelicanCtrl/gotoPos.h"

using namespace TooN;

double MAV::speed = 1;

MAV::MAV()
{
    pos = makeVector(0,0,10);
    speed = 1;
}

void MAV::Init(ros::NodeHandle *nh_, bool simulation_)
{
    nh = nh_;
    simulation = simulation_;

    if(!simulation)
    {
        gotoPosService = nh->serviceClient<PelicanCtrl::gotoPos>("/PelicanCtrl/gotoPos");
        atGoalSub = nh->subscribe("/PelicanCtrl/at_goal", 10, &MAV::atGoalCallback, this);
        //test();
    }
}

void MAV::test()
{
    SetGoal(makeVector(0,0,-5));
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

    glColor3f(0,0,1);
    glLineWidth(2);
    double l=0.25;
    glBegin(GL_LINES);
    Vector<3> p1,p2,p3,p4;
    p1=pos+makeVector(-l+r(),+r(),0);
    p2=pos+makeVector(+l+r(),+r(),0);
    p3=pos+makeVector(+r(),-l+r(),0);
    p4=pos+makeVector(+r(),+l+r(),0);
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

void MAV::SetGoal(TooN::Vector<3> goalpos)
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
