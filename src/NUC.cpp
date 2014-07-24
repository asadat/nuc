
#include "NUC.h"
#include "GL/glut.h"
#include "CNode.h"
#include "TooN/TooN.h"
#include "DepthFirstStrategy.h"
#include "LawnmowerStrategy.h"
#include "ShortCutStrategy.h"
#include "InterestingnessSensor.h"
#include "TestStrategy.h"
#include "NUCParam.h"
//#include "HuskyInterface.h"


FILE *NUC::logFile = NULL;
std::string NUC::logFileName = std::string("");

#define RAND(x,y) (x+((double)(rand()%1000)*0.001*(y-x)))
//#define AREA_LENGTH 16
//#define AREA_CX 0
//#define AREA_CY 0

using namespace TooN;

NUC * NUC::instance = NULL;
//bool NUC::simulation = true;

NUC::NUC(int argc, char **argv):nh("NUC")
{
    ros::NodeHandle private_node_handle_("~");
    NUCParam::GetParams(private_node_handle_);

    if(NUCParam::logging)
    {
        time_t now = time(0);
        char* dt = ctime(&now);
        logFileName = std::string(dt);
        for(unsigned int i=0; i<logFileName.length();i++)
        {
            printf("%d ",logFileName[i]);
            if(logFileName[i] == ' ' || logFileName[i] == ':' || logFileName[i] == 10)
                logFileName[i] = '_';
        }

        logFileName = NUCParam::log_folder+"/"+logFileName+".log";

        ROS_INFO("log file path: %s", logFileName.c_str());
        logFile = fopen(logFileName.c_str(), "w");
    }

    //bVisEnabled = true;
    //double area_length = 0;

//    NUCParam::simulation;
//    private_node_handle_.param<bool>("simulation", simulation, true);
//    private_node_handle_.param<bool>("visualization", bVisEnabled, true);
//    private_node_handle_.param<int>("branching_sqrt",CNode::bf_sqrt,2);
//    private_node_handle_.param<double>("speed",MAV::speed,1.0);
//    private_node_handle_.param<double>("area_length",area_length,16);

    int traversalStrategy=-1;
    std::string strategy_str;
    private_node_handle_.param("strategy", strategy_str, std::string("lm"));

    if(strategy_str == "lm")
    {
        traversalStrategy = 2;
    }
    else if(strategy_str == "df")
    {
        traversalStrategy = 0;
    }
    else if(strategy_str == "sc")
    {
        traversalStrategy = 1;
    }
    else if(strategy_str == "ts")
    {
        ROS_INFO("test strategy");

        traversalStrategy = 3;
    }

    mav.Init(&nh, NUCParam::simulation);

    if(!NUCParam::simulation)
    {
        InterestingnessSensor::Instance(&nh);
        //HuskyInterafce::Instance(&nh);
    }

    glutInit(&argc, argv);
    area = TooN::makeVector(NUCParam::cx-0.5*NUCParam::area_length,NUCParam::cy-0.5*NUCParam::area_length,
                            NUCParam::cx+0.5*NUCParam::area_length,NUCParam::cy+0.5*NUCParam::area_length);
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
    else if(traversalStrategy == 3)
    {
        traversal = new TestStrategy(tree);
    }
    else
    {
        traversal = new LawnmowerStrategy(tree);
    }

    ROS_INFO("test strategy %d", traversalStrategy);

    //For simulating interestingness
    if(NUCParam::simulation)
    {
        PopulateTargets();
        MarkNodesInterestingness();
    }
    //

    StartTraversing();   
}

NUC::~NUC()
{
    if(NUCParam::logging)
    {
        fclose(logFile);
    }
}

void NUC::PopulateTargets()
{
    srand(time(NULL));
    int n=5;
    double l =1*NUCParam::min_footprint;
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

     glLineWidth(5);
     glBegin(GL_LINES);
     glColor3f(1,0,0);
     glVertex3f(0,0,0.1);
     glVertex3f(1,0,0.1);

     glColor3f(0,1,0);
     glVertex3f(0,0,0.1);
     glVertex3f(0,1,0.1);

     glColor3f(0,0,1);
     glVertex3f(0,0,0.1);
     glVertex3f(0,0,1);
     glEnd();


     tree->glDraw();
     mav.glDraw();
     traversal->glDraw();

     TooN::Vector<2> c = TooN::makeVector(NUCParam::cx, NUCParam::cy);
     TooN::Matrix<2,2,double> rot = TooN::Data(cos(NUCParam::area_rotation*D2R), sin(NUCParam::area_rotation*D2R),
                                      -sin(NUCParam::area_rotation*D2R), cos(NUCParam::area_rotation*D2R));

     //ROS_INFO("%f\t%f\n%f\t%f", rot[0][0], rot[0][1], rot[1][0], rot[1][1]);

     if(NUCParam::simulation)
     {
         for(unsigned int i=0; i<targets.size(); i++)
         {
             TooN::Vector<2,double> p1,p2,v1,v2,v3,v4;
             p1[0] = targets[i][0];
             p1[1] = targets[i][1];
             p2[0] = targets[i][2];
             p2[1] = targets[i][3];

             v1 = p1;
             v2 = makeVector(p1[0], p2[1]);
             v3 = p2;
             v4 = makeVector(p2[0],p1[1]);

             Vector<2,double> r1 = c + rot*(v1-c);
             Vector<2,double> r2 = c + rot*(v2-c);
             Vector<2,double> r3 = c + rot*(v3-c);
             Vector<2,double> r4 = c + rot*(v4-c);


             glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
             glColor3f(0.2,1,0.2);
             glBegin(GL_POLYGON);
             glVertex3f(r1[0],r1[1], 0.11);
             glVertex3f(r2[0],r2[1], 0.11);
             glVertex3f(r3[0],r3[1], 0.11);
             glVertex3f(r4[0],r4[1], 0.11);
             glVertex3f(r1[0],r1[1], 0.11);

             glEnd();

         }
     }
}

void NUC::StartTraversing()
{
   ROS_INFO("Traverse starting...");
   startTime = ros::Time::now();
   LOG("starttime: %f \n", startTime.toSec());
   SetNextGoal();
   SAVE_LOG();
   mav.SetGoal(curGoal->GetMAVWaypoint(), true);
}

void NUC::SetNextGoal()
{
    curGoal = traversal->GetNextNode();
    if(curGoal != NULL)
    {
        LOG("NEXT_WAY_POINT: %f %f %f %d %d %d %f %f %f %f \n", curGoal->pos[0], curGoal->pos[1], curGoal->pos[2], curGoal->depth, curGoal->waiting, curGoal->isInteresting,
            curGoal->footPrint[0], curGoal->footPrint[1], curGoal->footPrint[2], curGoal->footPrint[3]);

        ROS_INFO("NEXT_WAY_POINT: %f %f %f \n", curGoal->pos[0], curGoal->pos[1], curGoal->pos[2]);
    }
}

void NUC::OnReachedGoal()
{
   // ROS_INFO("Reached Goal");
    if(VisitGoal())
    {
        SetNextGoal();
        if(curGoal == NULL)
        {
          //ROS_INFO("Finished ...");
          OnTraverseEnd();
        }
        else
        {
            //ROS_INFO("New goal ... depth %d", curGoal->depth);
            mav.SetGoal(curGoal->GetMAVWaypoint());
        }
    }
}

void NUC::OnTraverseEnd()
{
    endTime = ros::Time::now();
    ROS_INFO("Coverage duration: %f\n", (endTime-startTime).toSec());
    LOG("DURATION %f\n", (endTime-startTime).toSec());
    SAVE_LOG();

    exit(0);
}

bool NUC::VisitGoal()
{
    static ros::Time sensingStart = ros::Time::now();

    if(!curGoal->visited)
    {

        ROS_INFO("start sensing ....");
        LOG("SENSING_START %f \n", sensingStart.toSec());
        sensingStart = ros::Time::now();
        curGoal->visited = true;
    }

    if(NUCParam::simulation)
    {
        // In simulations it uses the precomputed interestingness
        curGoal->SetIsInteresting(curGoal->trueIsInteresting);
        //simulating interestingness
        for(unsigned int i=0; i<curGoal->children.size();i++)
            curGoal->children[i]->SetIsInteresting(curGoal->children[i]->trueIsInteresting);

        return true;
    }
    else
    {
        if((ros::Time::now()-sensingStart).toSec() > NUCParam::sensingTime)
        {
            bool curNodeInterest = false;
            //in real experiments it uses the image data to decide
            int grd_s = NUCParam::bf_sqrt;

            TooN::Matrix<10,10,int> grd_int = TooN::Zeros;
            InterestingnessSensor::Instance()->GetInterestingnessGrid(grd_int, grd_s);

            for(unsigned int i=0; i<curGoal->children.size();i++)
            {                
                CNode * nd = curGoal->children[i];
                nd->SetIsInteresting((grd_int[grd_s-nd->grd_y-1][nd->grd_x]>0));
                LOG("INTERSTINGNESS %d %d %d %d\n", nd->grd_x, nd->grd_y, nd->IsNodeInteresting(), grd_int[grd_s-nd->grd_y-1][nd->grd_x]);

                curNodeInterest = curNodeInterest || (grd_int[grd_s-nd->grd_y-1][nd->grd_x]>0);
            }

            curGoal->SetIsInteresting(curNodeInterest);

            if(curNodeInterest && curGoal->IsLeaf())
            {
                sensor_msgs::NavSatFix gpsTmp = mav.GetLastGPSLocation();
                //HuskyInterafce::Instance()->SendWaypoint(gpsTmp);
                LOG("WAYPOINT_TO_HUSKY %f %f %f", gpsTmp.latitude, gpsTmp.longitude, gpsTmp.altitude);
            }

            SAVE_LOG();

            return true;
        }
        else
        {
            ROS_INFO_THROTTLE(0.5,"Sensing ...");
        }

    }

    return false;
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

            //if(!simulation)
            //{
            //TooN::Matrix<10,10,int> grd_int = TooN::Zeros;
            //InterestingnessSensor::Instance()->GetInterestingnessGrid(grd_int, CNode::bf_sqrt);
            //}
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
