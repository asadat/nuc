#include "MAV.h"
#include "GL/glut.h"

using namespace TooN;

double MAV::speed = 1;

MAV::MAV()
{
    pos = makeVector(0,0,10);
    speed = 1;
}

MAV::~MAV()
{

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
    glColor3f(0,0,1);
    glLineWidth(2);
    double l=1;
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
    circle(p1[0]+r(),p1[1]+r(),p1[2], 0.5, 50);
    circle(p2[0]+r(),p2[1]+r(),p2[2], 0.5, 50);
    circle(p3[0]+r(),p3[1]+r(),p3[2], 0.5, 50);
    circle(p4[0]+r(),p4[1]+r(),p4[2], 0.5, 50);
}

void MAV::SetGoal(TooN::Vector<3> goalpos)
{
    goal = goalpos;
    toGoalNorm = (goal-pos);
    normalize(toGoalNorm);
    atGoal = false;

}

void MAV::Update(double dt)
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
