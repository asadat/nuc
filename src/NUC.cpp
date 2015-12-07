
#include "opencv2/stitching/stitcher.hpp"
#include "NUC.h"
#include "GL/glut.h"
#include "CNode.h"
#include "TooN/TooN.h"
#include "SearchCoverageStrategy.h"
#include "InterestingnessSensor.h"
#include "NUCParam.h"

#define AREA(r) (fabs(r[0]-r[2]) * fabs(r[1]-r[3]))

FILE *NUC::logFile = NULL;
std::string NUC::logFileName = std::string("");

using namespace TooN;

NUC * NUC::instance = NULL;

NUC::NUC(int argc, char **argv):nh("NUC")
{
    traversal = NULL;
    visitedFalsePositives = 0;
    drawPath = false;
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

    mav.Init(&nh, NUCParam::simulation);

    if(!NUCParam::simulation && !NUCParam::interesting_simulation)
    {
        InterestingnessSensor::Instance(&nh);
    }

    glutInit(&argc, argv);
    area = TooN::makeVector(NUCParam::cx-0.5*NUCParam::area_length,NUCParam::cy-0.5*NUCParam::area_length,
                            NUCParam::cx+0.5*NUCParam::area_length,NUCParam::cy+0.5*NUCParam::area_length);
    tree = new CNode(area);
    tree->PropagateDepth();

    if(NUCParam::simulation || NUCParam::interesting_simulation)
    {
        LoadPriorFromFile();        
    }

    assert(NUCParam::strategy == "scs");
    traversal = new SearchCoverageStrategy(tree);

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

float normal_pdf(float x, float m, float s)
{
    static const float inv_sqrt_2pi = 0.3989422804014327;
    float a = (x - m) / s;

    return inv_sqrt_2pi / s * std::exp(-0.5f * a * a);
}

void NUC::LoadPriorFromFile()
{
//    ROS_INFO("PDF: %f %f", normal_pdf(0,0,1), normal_pdf(10,0,10));
//    set<CNode*> leaves;
//    tree->GetLeavesInRange(leaves, NUCParam::area_length*1.5, makeVector(0,0,0));

//    for(auto it=leaves.begin(); it!= leaves.end(); it++)
//    {
//        Vector<3> wp = (*it)->GetMAVWaypoint();
//        (*it)->SetPrior(0.5);
//        float flip = 400000*(normal_pdf(wp[0]+wp[1], 0, 20)*normal_pdf(wp[1]-wp[0], 0, 15));
//        (*it)->imgPrior = (flip > 0.2?1.0:0.5);
//    }

//    return;



    std::string fn = NUCParam::nuc_dir+"/";
    fn += NUCParam::prior_file_name;
    ROS_INFO("Opening prior %s", fn.c_str());
    cv::Mat img = cv::imread(fn.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat priorImg;

    int d =-1;
    tree->ComputeDepth(d);
    int w = pow(2,d);

    cv::Size s(w,w);
    cv::resize(img, priorImg, s);
    CNode * bl = tree->GetNearestLeaf(makeVector(area[0],area[1],0));
    int dir = 1;
    for(int i=0; i<w; i++)
    {
        double pr_r = priorImg.at<uchar>(w-i, (dir<0)?w-1:0)*(1.0/255.0);
        bl->SetPrior(0.5);
        bl->imgPrior = pr_r;

        for(int j=(dir>0)?1:w-2; (dir>0)?(j<w):(j>=0); j+=dir)
        {
            bl = bl->GetNeighbourLeaf((dir<0), (dir>0), false, false);
            double pr_round = priorImg.at<uchar>(w-i-1,j)*(1.0/255.0);
            bl->SetPrior(0.5);
            bl->imgPrior = pr_round;
        }

        if(i<w-1)
        {
            bl = bl->GetNeighbourLeaf(false, false, true, false);
        }


        dir *= -1;

    }

    tree->RecomputePrior();
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
        colors.push_back(makeVector(0.5,0.5,0.5));
        colors.push_back(makeVector(0,1,1));
        colors.push_back(makeVector(1,0,1));
        colors.push_back(makeVector(0,1,0));
        colors.push_back(makeVector(0,0,1));
        colors.push_back(makeVector(1,1,0));
        colors.push_back(makeVector(1,0,0));

        colors.push_back(makeVector(160/255.0, 109/255.0, 21/255.0));
        colors.push_back(makeVector(62/255.0, 102/255.0, 106/255.0));


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

    while(targets.size() < (unsigned int)NUCParam::patches)
    {
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


        if(fabs(r[0]-r[2]) < cellW || fabs(r[1]-r[3]) < cellW)
                continue;

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
            //ROS_INFO("STRATEGY: RECT %f %f %f %f", r[0], r[1], r[2], r[3]);
        }
        else
        {
            n++;
            if(n>10)
                targets.clear();

        }

    }

    for(unsigned int i=0; i < targets.size(); i++)
        ROS_INFO("target:%d area:%.2f %.2f %.2f %.2f %.2f", i, AREA(targets[i]), targets[i][0],targets[i][1],targets[i][2],targets[i][3]);

}

void NUC::MarkNodesInterestingness()
{
    for(unsigned int i=0; i<targets.size(); i++)
    {
        tree->PropagateInterestingness(targets[i]);
        tree->InitializePrior(targets[i]);
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

     glVertex3f(-w/2, -w/2, 0.0);
     glVertex3f( w/2, -w/2, 0.0);
     glVertex3f( w/2, w/2, 0.0);
     glVertex3f(-w/2, w/2, 0.0);
     glVertex3f(-w/2, -w/2, 0.0);

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


     if(drawPath && pathHistory.size() > 2 && !CNode::drawCoverage)
     {
         glColor3f(0,0,0);
         glLineWidth(2);
         glBegin(GL_LINES);
         for(unsigned int i=1; i+2<pathHistory.size();i++)
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

         glColor3f(0,0,0);
         glPointSize(3);
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

     if(traversal)
         traversal->glDraw();

     TooN::Vector<2> c = TooN::makeVector(NUCParam::cx, NUCParam::cy);
     TooN::Matrix<2,2,double> rot = TooN::Data(cos(NUCParam::area_rotation*D2R), sin(NUCParam::area_rotation*D2R),
                                      -sin(NUCParam::area_rotation*D2R), cos(NUCParam::area_rotation*D2R));

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



             glLineWidth(8);
             glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

             glColor4f(0.8,0.4,0.0,1);
             glBegin(GL_POLYGON);
             glVertex3f(r1[0],r1[1], 0.7511);
             glVertex3f(r2[0],r2[1], 0.7511);
             glVertex3f(r3[0],r3[1], 0.7511);
             glVertex3f(r4[0],r4[1], 0.7511);
             glVertex3f(r1[0],r1[1], 0.7511);

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
    //ROS_INFO("calling GetNextNode()");
    curGoal = traversal->GetNextNode();

    //ROS_INFO("returning from GetNextNode()");

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
    ros::Time t0 = ros::Time::now();

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
            if(!NUCParam::bypass_controller)
                mav.SetGoal(curGoal->GetMAVWaypoint());

            double dt = (ros::Time::now() - t0).toSec();
            ROS_WARN("Planning Time: %f", dt);
        }
    }
}

void NUC::OnTraverseEnd()
{
    double eps = 0.1;
    double asclength = 0;
    double desclength = 0;
    double xylength = 0;

    int UFN=0;
    tree->GetUnvisitedFalseNegatives(UFN);

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
    ROS_INFO("STRATEGY:%s PATCHES:%d PERCENT:%f DURATION: %f LENGTH %f ASC: %f DESC: %f Z_LENGTH: %f XY_LENGTH: %f Prob_ratio:%.2f VFP: %d UFN:%d\n",
             NUCParam::strategy.c_str(), NUCParam::patches, NUCParam::percent_interesting, (endTime-startTime).toSec(), traverseLength,
             asclength, desclength, asclength+desclength, xylength, NUCParam::prob_r, visitedFalsePositives, UFN);
    NUC_LOG("STRATEGY:%s PATCHES:%d PERCENT:%f DURATION %f LENGTH %f ASC: %f DESC: %f Z_LENGTH: %f XY_LENGTH: %f Prob_ratio:%.2f VFP: %d UFN:%d\n",
        NUCParam::strategy.c_str(), NUCParam::patches, NUCParam::percent_interesting, (endTime-startTime).toSec(), traverseLength,
        asclength, desclength, asclength+desclength, xylength, NUCParam::prob_r, visitedFalsePositives, UFN);
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
        NUC_LOG("SENSING_START %f \n", sensingStart.toSec());
        sensingStart = ros::Time::now();
        curGoal->SetTreeVisited(true);
    }

    if(NUCParam::simulation)
    {       
        traversal->ReachedNode(curGoal);
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
            ros::spinOnce();          
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
        updateKey = false;
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
    else if(key['`'])
    {
        drawPath = !drawPath;
        //updateKey = false;

    }
    else if(key['5'])
    {
        mav.JumpToGoal();
        updateKey = false;
    }


    traversal->hanldeKeyPressed(key, updateKey);
}
