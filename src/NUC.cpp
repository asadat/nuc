
#include "opencv2/stitching/stitcher.hpp"
#include "NUC.h"
#include "GL/glut.h"
#include "CNode.h"
#include "TooN/TooN.h"
#include "DepthFirstStrategy.h"
#include "LawnmowerStrategy.h"
#include "ShortCutStrategy.h"
#include "InterestingnessSensor.h"
#include "HilbertStrategy.h"
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
    //runCircleDetection();
    //runPhotoStitcher();
    traverseLength = 0;
    isOver = false;
    sim_running = true;
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
    //std::string strategy_str;
    //private_node_handle_.param("strategy", strategy_str, std::string("lm"));

    if(NUCParam::strategy == "lm")
    {
        traversalStrategy = 2;
    }
    else if(NUCParam::strategy == "df")
    {
        traversalStrategy = 0;
    }
    else if(NUCParam::strategy == "sc")
    {
        traversalStrategy = 1;
    }
    else if(NUCParam::strategy == "ts")
    {
        ROS_INFO("test strategy");

        traversalStrategy = 3;
    }else if(NUCParam::strategy == "hi")
    {
        ROS_INFO("hilber strategy");

        traversalStrategy = 4;
    }

    mav.Init(&nh, NUCParam::simulation);

    if(!NUCParam::simulation && !NUCParam::interesting_simulation)
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
    else if(traversalStrategy == 4)
    {
        traversal = new HilbertStrategy(tree);
    }
    else
    {
        traversal = new LawnmowerStrategy(tree);
    }

    ROS_INFO("test strategy %d", traversalStrategy);

    //For simulating interestingness
    if(NUCParam::simulation || NUCParam::interesting_simulation)
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

    Cleanup();
}

void NUC::Cleanup()
{
    if(tree)
        delete tree;

    tree = NULL;
}

void NUC::runPhotoStitcher()
{
    ros::Time t = ros::Time::now();
    std::vector<cv::Mat > imgs;
    for(int i=1; i<= 20; i++)
    {
        std::ostringstream stream;
        stream << "/home/autolab/hydro_workspace/src/nuc/imgs/" << i << ".jpg";
        std::string fn = stream.str();

        ROS_INFO("%s", fn.c_str());
        cv::Mat im = cv::imread(fn);
        imgs.push_back(im);
    }

    cv::Stitcher stitcher = cv::Stitcher::createDefault(true);

    stitcher.setWarper(new cv::PlaneWarper());
    stitcher.setFeaturesFinder(new cv::detail::SurfFeaturesFinder(1000,3,4,3,4));
    stitcher.setRegistrationResol(0.1);
    stitcher.setSeamEstimationResol(0.1);
    stitcher.setCompositingResol(1);
    stitcher.setPanoConfidenceThresh(1);
    stitcher.setWaveCorrection(false);
    stitcher.setWaveCorrectKind(cv::detail::WAVE_CORRECT_HORIZ);
    stitcher.setFeaturesMatcher(new cv::detail::BestOf2NearestMatcher(false,0.3));
    stitcher.setBundleAdjuster(new cv::detail::BundleAdjusterRay());
    //Stitcher::Status status = Stitcher::ERR_NEED_MORE_IMGS;
    cv::Mat pano;
    cv::Stitcher::Status status = stitcher.stitch(imgs, pano);

    if(status == cv::Stitcher::OK)
    {
        cv::imwrite("/home/autolab/hydro_workspace/src/nuc/imgs/pano.jpg", pano);
    }
    else
    {
        ROS_INFO("Can't stitch images, error code = %d", int(status));
    }

    ROS_INFO("dtime: %.2f", (ros::Time::now()-t).toSec());
    exit(0);

}

TooN::Vector<3> NUC::GetColor(double h)
{
    Vector<3> c;
    double small_dh=0.01;
    static std::vector<Vector<3> > colors;
    static std::vector<Vector<2> > h2c;

    if(colors.empty())
    {
        colors.push_back(makeVector(0,0,0));
        colors.push_back(makeVector(1,0,0));
        colors.push_back(makeVector(0,1,0));
        colors.push_back(makeVector(0,0,1));
        colors.push_back(makeVector(0,1,1));
        colors.push_back(makeVector(1,0,1));
        colors.push_back(makeVector(1,1,0));
        colors.push_back(makeVector(0.5,0.5,0.5));
        std::reverse(colors.begin(), colors.end());
    }

    if(h2c.empty())
    {
        Vector<2> i;
        i[0] = h;
        i[1] = 0;
        c = colors[0];
        h2c.push_back(i);
    }
    else
    {
        bool flag = false;
        for(unsigned int i=0; i<h2c.size(); i++)
        {
            if(fabs(h2c[i][0]-h) < small_dh)
            {
                c = colors[(int)h2c[i][1]];
                flag = true;
                break;
            }
        }

        if(!flag)
        {
            Vector<2> hc;
            hc[0] = h;
            hc[1] = h2c.size();
            h2c.push_back(hc);
            c = colors[h2c.size()];
        }
    }

    return c;
}

bool NUC::RectIntersect(Rect r, Rect d)
{
    double ep = 0.1;
    if(r[0]+ep > d[2] || r[2]-ep < d[0] || r[1]+ep > d[3] || r[3]-ep < d[1])
        return false;
    return true;
}

void NUC::PopulateTargets()
{
    srand(time(NULL));
    //double xy_ratio = 1/5.0;

    CNode* leaf = tree->GetNearestLeaf(makeVector(0,0,0));
    Rect lr;
    double cellW;
    if(leaf)
    {
        lr = leaf->GetFootPrint();
        cellW = fabs(lr[2]-lr[0]);
    }

    int n=0;
    double patch = ((NUCParam::percent_interesting/100.0)* fabs(area[0]-area[2])*fabs(area[1]-area[3])/NUCParam::patches);
    //lx = floor(lx/(lr[2]-lr[0]))* (lr[2]-lr[0]);

    while(targets.size() < (unsigned int)NUCParam::patches)
    {
//        Rect r;
//        r[0] = RAND(area[0], area[2]-lx);
//        r[0] = floor(r[0]/(lr[2]-lr[0]))*(lr[2]-lr[0]);
//        r[1] = RAND(area[1], area[3]-lx/xy_ratio);
//        r[1] = floor(r[1]/(lr[3]-lr[1]))* (lr[3]-lr[1]);
//        r[2] = r[0]+lx;
//        r[3] = r[1]+lx/xy_ratio;

        Rect r;
        r[0] = RAND(area[0], area[2]-cellW);
        r[0] = floor(r[0]/cellW)*cellW;

        r[2] = RAND(r[0], area[2]);
        r[2] = ceil(r[2]/cellW)*cellW;

        double ly = patch / (r[2]-r[0]);
        double ly1 = ceil(ly/cellW)*cellW;
        double ly2 = floor(ly/cellW)*cellW;
        if(fabs(ly - ly1) < fabs(ly-ly2))
            ly = ly1;
        else
            ly = ly2;


        if(ly > area[3]-area[1])
            continue;

        r[1] = RAND(area[1], area[3]-ly);
        r[1] = floor(r[1]/cellW)*cellW;

        r[3] = r[1]+ly;


        r[0] = -13;
        r[1] = -14;
        r[2] = 10;
        r[3] = 9;

        bool flag=true;
        for(unsigned int i=0; i<targets.size(); i++)
        {
            if(RectIntersect(r,targets[i]))
            {
                flag = false;
                break;
            }
        }

        if(flag)
        {
            targets.push_back(r);
            n=0;
            ROS_INFO("STRATEGY: RECT %f %f %f %f", r[0], r[1], r[2], r[3]);
        }
        else
        {
            n++;
            if(n>10)
                targets.clear();

        }

    }
//    srand(time(NULL));
//    int n = 0.5 * NUCParam::area_length / NUCParam::min_footprint;
//    double l =2*NUCParam::min_footprint;
//    for(int i=0; i<n; i++)
//    {
//        Rect r;
//        r[0] = RAND(area[0], area[2]-l);
//        r[1] = RAND(area[1], area[3]-l);
//        r[2] = r[0]+ RAND(l*0.5,l*1.5);
//        r[3] = r[1]+ RAND(l*0.5,l*1.5);

//        targets.push_back(r);
//    }
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
    //ROS_INFO("rendering ....");
     double w = fabs(area[2]-area[0]);
     glLineWidth(4);
     glColor3f(0,0,0);
     glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
     glBegin(GL_POLYGON);

     glVertex3f(-w/2, -w/2, 0.1);     
     glVertex3f( w/2, -w/2, 0.1);     
     glVertex3f( w/2, w/2, 0.1);
     glVertex3f(-w/2, w/2, 0.1);
     glVertex3f(-w/2, -w/2, 0.1);

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


     if(pathHistory.size() > 2 && !CNode::drawCoverage)
     {
         glColor3f(0,0,0);
         glLineWidth(4);
         glBegin(GL_LINES);
         for(unsigned int i=0; i<pathHistory.size()-1;i++)
         {
             TooN::Vector<3> p1 =pathHistory[i+1];
             TooN::Vector<3> p2 =pathHistory[i];
             TooN::Vector<3> c1 = GetColor(p1[2]);
             TooN::Vector<3> c2 = GetColor(p2[2]);

             glColor3f(c1[0], c1[1], c1[2]);
             glVertex3f(p1[0],p1[1],p1[2]);
             glColor3f(c2[0], c2[1], c2[2]);
             glVertex3f(p2[0],p2[1],p2[2]);
         }
         glEnd();

         glColor3f(1,0,0);
         glPointSize(5);
         glBegin(GL_POINTS);
         for(unsigned int i=0; i<pathHistory.size();i++)
         {
             TooN::Vector<3> p =pathHistory[i];          
             glVertex3f(p[0],p[1],p[2]);

         }
         glEnd();
     }

     tree->glDraw();
     mav.glDraw();

     //if(isOver)
         traversal->glDraw();

     TooN::Vector<2> c = TooN::makeVector(NUCParam::cx, NUCParam::cy);
     TooN::Matrix<2,2,double> rot = TooN::Data(cos(NUCParam::area_rotation*D2R), sin(NUCParam::area_rotation*D2R),
                                      -sin(NUCParam::area_rotation*D2R), cos(NUCParam::area_rotation*D2R));

     //ROS_INFO("%f\t%f\n%f\t%f", rot[0][0], rot[0][1], rot[1][0], rot[1][1]);

     if(NUCParam::interesting_simulation)
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


             if(!CNode::drawCoverage)
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
             else
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

             glColor4f(0.2,1,0.2,0.4);
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
   NUC_LOG("starttime: %f \n", startTime.toSec());
   pathHistory.push_back(mav.GetPos());
   SetNextGoal();
   SAVE_LOG();
   mav.SetGoal(curGoal->GetMAVWaypoint(), true);

}

void NUC::SetNextGoal()
{    
    curGoal = traversal->GetNextNode();
    if(curGoal != NULL)
    {
        pathHistory.push_back(curGoal->GetMAVWaypoint());
        //traverseLength += sqrt((mav.GetPos()-curGoal->GetMAVWaypoint())*(mav.GetPos()-curGoal->GetMAVWaypoint()));
        NUC_LOG("NEXT_WAY_POINT: %f %f %f %d %d %d %f %f %f %f \n", curGoal->GetMAVWaypoint()[0], curGoal->GetMAVWaypoint()[1], curGoal->GetMAVWaypoint()[2], curGoal->depth, curGoal->waiting, curGoal->isInteresting,
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
          ROS_INFO("Finished ...");
          OnTraverseEnd();
        }
        else
        {
            //ROS_INFO("New goal ... depth %d", curGoal->depth);
            if(!NUCParam::bypass_controller)
                mav.SetGoal(curGoal->GetMAVWaypoint());
        }
    }
}

void NUC::OnTraverseEnd()
{
    double eps = 0.1;
    double asclength = 0;
    double desclength = 0;
    double xylength = 0;

    if(pathHistory.size()>1)
    {
        for(unsigned int i=0; i<pathHistory.size()-1; i++)
        {
            traverseLength += sqrt((pathHistory[i]-pathHistory[i+1])*(pathHistory[i]-pathHistory[i+1]));
            asclength +=  (pathHistory[i+1][2]-pathHistory[i][2] > eps)?pathHistory[i+1][2]-pathHistory[i][2]:0;
            desclength +=  (pathHistory[i+1][2]-pathHistory[i][2] < -eps)?-(pathHistory[i+1][2]-pathHistory[i][2]):0;
            xylength += sqrt((pathHistory[i+1][0]-pathHistory[i][0])*(pathHistory[i+1][0]-pathHistory[i][0])+
                             (pathHistory[i+1][1]-pathHistory[i][1])*(pathHistory[i+1][1]-pathHistory[i][1]));
        }
    }

    endTime = ros::Time::now();
    ROS_INFO("STRATEGY:%s PATCHES:%d PERCENT:%f DURATION: %f LENGTH %f ASC: %f DESC: %f Z_LENGTH: %f XY_LENGTH: %f\n",
             NUCParam::strategy.c_str(), NUCParam::patches, NUCParam::percent_interesting, (endTime-startTime).toSec(), traverseLength,
             asclength, desclength, asclength+desclength, xylength);
    NUC_LOG("STRATEGY:%s PATCHES:%d PERCENT:%f DURATION %f LENGTH %f ASC: %f DESC: %f Z_LENGTH: %f XY_LENGTH: %f\n",
        NUCParam::strategy.c_str(), NUCParam::patches, NUCParam::percent_interesting, (endTime-startTime).toSec(), traverseLength,
        asclength, desclength, asclength+desclength, xylength);
    SAVE_LOG();

    if(NUCParam::auto_exit)
    {
        Cleanup();
        exit(0);
    }

    isOver = true;
}

bool NUC::VisitGoal()
{
    static ros::Time sensingStart = ros::Time::now();

    if(!curGoal->visited)
    {

       // ROS_INFO("start sensing ....");
        NUC_LOG("SENSING_START %f \n", sensingStart.toSec());
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

        curGoal->propagateCoverage(curGoal->pos[2]);
        return true;
    }
    else
    {
        if((ros::Time::now()-sensingStart).toSec() > NUCParam::sensingTime)
        {
            bool curNodeInterest = false;
            //in real experiments it uses the image data to decide

            if(NUCParam::interesting_simulation)
            {
                // In simulations it uses the precomputed interestingness
                curGoal->SetIsInteresting(curGoal->trueIsInteresting);
                //simulating interestingness
                for(unsigned int i=0; i<curGoal->children.size();i++)
                    curGoal->children[i]->SetIsInteresting(curGoal->children[i]->trueIsInteresting);
            }
            else
            {
                int grd_s = NUCParam::bf_sqrt;

                TooN::Matrix<10,10,int> grd_int = TooN::Zeros;
                InterestingnessSensor::Instance()->GetInterestingnessGrid(grd_int, grd_s);

                if(!curGoal->children.empty())
                {
                    for(unsigned int i=0; i<curGoal->children.size();i++)
                    {
                        CNode * nd = curGoal->children[i];
                        nd->SetIsInteresting((grd_int[grd_s-nd->grd_y-1][nd->grd_x]>0));
                        NUC_LOG("INTERSTINGNESS %d %d %d %d %d\n", nd->grd_x, nd->grd_y, nd->IsNodeInteresting(), grd_int[grd_s-nd->grd_y-1][nd->grd_x], InterestingnessSensor::Instance()->sensingCounter);

                        curNodeInterest = curNodeInterest || (grd_int[grd_s-nd->grd_y-1][nd->grd_x]>0);
                    }
                }
                else
                {
                    for(int i=0; i< NUCParam::bf_sqrt; i++)
                        for(int j=0; j< NUCParam::bf_sqrt; j++)
                        {
                            curNodeInterest = curNodeInterest || (grd_int[j][i]>0);
                            NUC_LOG("INTERSTINGNESS %d %d %d %d %d\n", i, j, (grd_int[j][i]>0), grd_int[j][i], InterestingnessSensor::Instance()->sensingCounter);
                        }

                }
                curGoal->SetIsInteresting(curNodeInterest);

                if(curNodeInterest && curGoal->IsLeaf())
                {
                    sensor_msgs::NavSatFix gpsTmp = mav.GetLastGPSLocation();
                    //HuskyInterafce::Instance()->SendWaypoint(gpsTmp);
                    NUC_LOG("WAYPOINT_TO_HUSKY %f %f %f", gpsTmp.latitude, gpsTmp.longitude, gpsTmp.altitude);
                }
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
    if(isOver)
        return;

    static double rosFreq=15.0;
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
    if(NUCParam::bypass_controller || dtmav > 0.001)
    {
        lastTimeMav = ros::Time::now();
        if(!NUCParam::bypass_controller)
            mav.Update(dtmav);

        if(NUCParam::bypass_controller || mav.AtGoal())
        {
            OnReachedGoal();
        }
    }
}

void NUC::hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey)
{
    //updateKey = true;

    if(key['='])
    {
        MAV::ChangeSpeed(0.1);
        //updateKey = false;
    }
    else if(key['-'])
    {
        MAV::ChangeSpeed(-0.1);
        //updateKey = false;
    }
    else if(key['0'])
    {
        NUCParam::sim_running = !NUCParam::sim_running;
        //updateKey = false;
    }
    else if(key['1'])
    {
        CNode::drawEdges = !CNode::drawEdges;
        //updateKey = false;
    }
    else if(key['2'])
    {
        CNode::drawCoverage = !CNode::drawCoverage;
        //updateKey = false;

    }

    traversal->hanldeKeyPressed(key, updateKey);
}
