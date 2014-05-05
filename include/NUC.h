#ifndef _NUC_
#define _NUC_

#include "ros/ros.h"
#include "CNode.h"
#include "TraversalStrategy.h"
#include "DepthFirstStrategy.h"
#include "MAV.h"

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
    bool VisEnabled(){return bVisEnabled;}

    void hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey);

    static bool simulation;

private:

    NUC(int argc, char **argv);
    bool VisitGoal();
    void PopulateTargets();
    void MarkNodesInterestingness();

    static NUC* instance;

    ros::NodeHandle nh;

    Rect area;
    CNode* tree;
    TraversalStrategy * traversal;
    CNode* curGoal;

    MAV mav;

    bool bVisEnabled;

    std::vector<Rect> targets;

    ros::Time startTime;
    ros::Time endTime;

    static FILE* logFile;
    static std::string logFileName;

};

#endif
