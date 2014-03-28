
#include "NUC.h"
#include "GL/glut.h"
#include "CNode.h"
#include "TooN/TooN.h"
#include "DepthFirstStrategy.h"

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
    traversal = new DepthFirstStrategy(tree);
    StartTraversing();
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
     mav.glDraw();

}

void NUC::StartTraversing()
{
   curGoal = traversal->GetNextNode();
   mav.SetGoal(curGoal->GetPos());
}

void NUC::OnReachedGoal()
{
    VisitGoal();
    curGoal = traversal->GetNextNode();
    if(curGoal == NULL)
    {
      OnTraverseEnd();
    }
    else
    {
        mav.SetGoal(curGoal->GetPos());
    }
}

void NUC::OnTraverseEnd()
{

}

void NUC::VisitGoal()
{

}

void NUC::Update()
{
    static double rosFreq=15;
    static double ros_p = 1/rosFreq;
    static ros::Time lastTime = ros::Time::now();
    //static int i=0;
    double dt = (ros::Time::now()-lastTime).toSec();
    if( dt > ros_p)
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

        mav.Update(dt);
        if(mav.AtGoal())
        {
            OnReachedGoal();
        }
    }

}

void NUC::hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey)
{
    updateKey = true;

    if(key['='])
    {
        MAV::ChangeSpeed(0.5);
    }
    else if(key['-'])
    {
        MAV::ChangeSpeed(-0.5);
    }
}
