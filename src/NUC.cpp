
#include "NUC.h"
#include "GL/glut.h"
#include "CNode.h"
#include "TooN/TooN.h"
#include "DepthFirstStrategy.h"
#include "LawnmowerStrategy.h"
#include "ShortCutStrategy.h"
#include "InterestingnessSensor.h"

#define RAND(x,y) (x+((double)(rand()%1000)*0.001*(y-x)))
#define AREA_LENGTH 32
#define AREA_CX 0
#define AREA_CY 0


NUC * NUC::instance = NULL;
bool NUC::simulation = true;

NUC::NUC(int argc, char **argv):nh("NUC")
{
    bVisEnabled = true;


    int traversalStrategy=-1;

    for(int i=1; i<argc;i++)
    {
        if(strcmp(argv[i],"-real")==0)
        {
             simulation = false;
        }
        else if(strcmp(argv[i],"-novis")==0)
        {
            bVisEnabled = false;
        }
        else if(strcmp(argv[i],"-b")==0)
        {
           CNode::bf_sqrt = atoi(argv[++i]);
        }
        else if(strcmp(argv[i],"-s")==0)
        {
           MAV::speed = atoi(argv[++i]);
        }
        else if(strcmp(argv[i],"-dfs")==0)
        {
            traversalStrategy = 0;
        }
        else if(strcmp(argv[i],"-scs")==0)
        {
            traversalStrategy = 1;
        }
        else if(strcmp(argv[i],"-lms")==0)
        {
            traversalStrategy = 2;
        }

    }

    InterestingnessSensor::Instance(&nh);
    mav.Init(&nh, simulation);

    glutInit(&argc, argv);
    area = TooN::makeVector(AREA_CX-0.5*AREA_LENGTH,AREA_CY-0.5*AREA_LENGTH,AREA_CX+0.5*AREA_LENGTH,AREA_CY+0.5*AREA_LENGTH);
    tree = new CNode(area);
    tree->PropagateDepth();

    if(traversalStrategy == 0)
    {
        traversal = new DepthFirstStrategy(tree);
    }
    else if(traversalStrategy == 1)
    {
        traversal = new ShortCutStrategy(tree);
    }
    else
    {
        traversal = new LawnmowerStrategy(tree);
    }


    //For simulating interestingness
    if(simulation)
    {
        PopulateTargets();
        MarkNodesInterestingness();
    }
    //

    StartTraversing();
}

NUC::~NUC()
{

}

void NUC::PopulateTargets()
{
    srand(time(NULL));
    int n=10;
    double l =5*CNode::minFootprintWidth;
    for(int i=0; i<n; i++)
    {
        Rect r;
        r[0] = RAND(area[0], area[2]-l);
        r[1] = RAND(area[1], area[3]-l);
        r[2] = r[0]+l;
        r[3] = r[1]+l;

        targets.push_back(r);
    }
}

void NUC::MarkNodesInterestingness()
{
    for(unsigned int i=0; i<targets.size(); i++)
    {
        tree->PropagateInterestingness(targets[i]);
    }
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
     traversal->glDraw();

     if(simulation)
     {
         for(unsigned int i=0; i<targets.size(); i++)
         {
             glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
             glColor3f(0.2,1,0.2);
             glBegin(GL_QUADS);
             glVertex3f(targets[i][0],targets[i][1], 0.11);
             glVertex3f(targets[i][0],targets[i][3], 0.11);
             glVertex3f(targets[i][2],targets[i][3], 0.11);
             glVertex3f(targets[i][2],targets[i][1], 0.11);
             glEnd();

         }
     }
}

void NUC::StartTraversing()
{
   startTime = ros::Time::now();
   curGoal = traversal->GetNextNode();
   mav.SetGoal(curGoal->GetPos());
}

void NUC::OnReachedGoal()
{
    //ROS_INFO("Reached Goal");

    VisitGoal();
    curGoal = traversal->GetNextNode();
    if(curGoal == NULL)
    {
      ROS_INFO("Finished ...");
      OnTraverseEnd();
    }
    else
    {
        //ROS_INFO("New goal ... depth %d", curGoal->depth);
        mav.SetGoal(curGoal->GetPos());
    }
}

void NUC::OnTraverseEnd()
{
    endTime = ros::Time::now();
    ROS_INFO("Coverage duration: %f\n", (endTime-startTime).toSec());
    exit(0);
}

void NUC::VisitGoal()
{
    curGoal->visited = true;

    if(simulation)
    {
        // In simulations it uses the precomputed interestingness
        curGoal->SetIsInteresting(curGoal->trueIsInteresting);
        //simulating interestingness
        for(unsigned int i=0; i<curGoal->children.size();i++)
            curGoal->children[i]->SetIsInteresting(curGoal->children[i]->trueIsInteresting);
    }
    else
    {
        // in real experiments it uses the image data to decide
        //InterestingnessSensor::Instance()->GetInterestingness(for all children);
    }
}

void NUC::Update()
{
    static double rosFreq=15;
    static double ros_p = 1/rosFreq;
    static ros::Time lastTime = ros::Time::now();
    static ros::Time lastTimeMav = ros::Time::now();

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
    }

    double dtmav = (ros::Time::now()-lastTimeMav).toSec();
    if(dtmav > 0.001)
    {
        lastTimeMav = ros::Time::now();
        mav.Update(dtmav);
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
        MAV::ChangeSpeed(0.1);
    }
    else if(key['-'])
    {
        MAV::ChangeSpeed(-0.1);
    }
}
