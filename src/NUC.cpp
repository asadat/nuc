
#include "NUC.h"
#include "GL/glut.h"
#include "CNode.h"
#include "TooN/TooN.h"

NUC * NUC::instance = NULL;

NUC::NUC(int argc, char **argv)
{
    bVisEnabled = true;

    for(int i=1; i<argc;i++)
    {
        if(strcmp(argv[i],"-novis")==0)
        {
            bVisEnabled = false;
        }
        else if(strcmp(argv[i],"-b")==0)
        {
           CNode::bf_sqrt = atoi(argv[++i]);
        }

    }

    glutInit(&argc, argv);

    tree = new CNode(TooN::makeVector(-32,-32,32,32));
}

NUC::~NUC()
{

}

void NUC::glDraw()
{
     float w=64;
     glLineWidth(1);
     glColor3f(0.3,0.3,0.3);
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

     tree->glDraw();


}

void NUC::StartTraversing()
{
   curGoal = traversal->GetNextNode();
}

void NUC::Update()
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

