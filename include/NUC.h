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

    void Update();
    void glDraw();
    bool VisEnabled(){return bVisEnabled;}

    void hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey);
private:

    NUC(int argc, char **argv);
    void VisitGoal();

    static NUC* instance;

    ros::NodeHandle nh;

//    ros::Subscriber gpsPos_sub;
//    ros::Subscriber gpsPose_sub;

    CNode* tree;
    TraversalStrategy * traversal;
    CNode* curGoal;

    MAV mav;

    bool bVisEnabled;


};

#endif