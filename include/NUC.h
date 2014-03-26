#ifndef _NUC_
#define _NUC_

#include "ros/ros.h"
#include "CNode.h"
#include "TraversalStrategy.h"
#include "DepthFirstStrategy.h"

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

    void Update();
    void glDraw();
    bool VisEnabled(){return bVisEnabled;}

private:

    NUC(int argc, char **argv);
    static NUC* instance;

    ros::NodeHandle nh;

//    ros::Subscriber gpsPos_sub;
//    ros::Subscriber gpsPose_sub;

    CNode* tree;
    TraversalStrategy * traversal;
    CNode* curGoal;

    bool bVisEnabled;


};

#endif
