#ifndef _NUC_
#define _NUC_

#include "ros/ros.h"
#include "CNode.h"
#include "TraversalStrategy.h"
#include "DepthFirstStrategy.h"
#include "MAV.h"

#define LOG( ... ) if(NUCParam::logging){/*printf( __VA_ARGS__ );*/fprintf(NUC::logFile,"%f ", ros::Time::now().toSec());fprintf(NUC::logFile, __VA_ARGS__ );fflush(NUC::logFile);}
#define SAVE_LOG() if(NUCParam::logging){fclose(NUC::logFile);NUC::logFile = fopen(NUC::logFileName.c_str(), "a+");}


class NUC
{
public:

    ~NUC();

    static NUC * Instance(int argc=0, char **argv=NULL)
    {
        if(instance == NULL)
        {
            instance = new NUC(argc, argv);
        }

        return instance;
    }

    void StartTraversing();
    void OnReachedGoal();
    void OnTraverseEnd();
    void SetNextGoal();

    void Update();
    void glDraw();
    bool VisEnabled(){return NUCParam::visualization;}

    void hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey);

    //static bool simulation;
    static FILE* logFile;
    bool isOver;

private:

    NUC(int argc, char **argv);
    bool VisitGoal();
    void PopulateTargets();
    void MarkNodesInterestingness();
    bool RectIntersect(Rect r, Rect d);
    TooN::Vector<3> GetColor(double h);
    void runPhotoStitcher();
    void Cleanup();

    static NUC* instance;

    ros::NodeHandle nh;

    Rect area;
    CNode* tree;
    TraversalStrategy * traversal;
    CNode* curGoal;

    std::vector<TooN::Vector<3> > pathHistory;
    MAV mav;

    //bool bVisEnabled;

    std::vector<Rect> targets;

    ros::Time startTime;
    ros::Time endTime;
    double traverseLength;

    bool sim_running;

    static std::string logFileName;

};

#endif
