#ifndef _CNODE_
#define _CNODE_
#include <vector>
#include "TooN/TooN.h"

typedef TooN::Vector<4> Rect;
class LawnmowerStrategy;
class DepthFirstStrategy;
class ShortCutStrategy;
class NUC;


class CNode
{
public:
    CNode(Rect target_foot_print);
    ~CNode();

    // coverage related methods
    void PopulateChildren();

    // tree related methods
    CNode* CreateChildNode(Rect fp);

    bool IsNodeInteresting(){return isInteresting;}// || trueIsInteresting;}
    void SetIsInteresting(bool interesting){isInteresting=interesting;}
    TooN::Vector<3> GetPos(){return pos;}
    TooN::Vector<3> GetMAVWaypoint();
    void glDraw();


    CNode* GetNearestLeaf(TooN::Vector<3> p);
    void GetNearestLeafAndParents(TooN::Vector<3> p, std::vector<CNode*> & list);

    Rect GetFootPrint(){return footPrint;}

    bool IsLeaf(){return children.empty();}

    static float fov;
    static int bf_sqrt;
    //static float minFootprintWidth;

    static TooN::Vector<3> Rotation2D(TooN::Vector<3> v, double deg, TooN::Vector<2> c);

    static TooN::Vector<2> Rotation2D(TooN::Vector<2> v, double deg, TooN::Vector<2> c);

private:

    std::vector<CNode*> children;

    //this is used for simulation only
    bool trueIsInteresting;
    bool PropagateInterestingness(Rect r);
    void PropagateDepth();

    bool VisitedInterestingDescendentExists();

    CNode *parent;

    bool visited;
    bool waiting;

    int grd_x, grd_y;
    int depth;
    Rect footPrint;
    TooN::Vector<3> pos;
    bool isInteresting;

    friend class LawnmowerStrategy;
    friend class DepthFirstStrategy;
    friend class ShortCutStrategy;
    friend class NUC;
};


#endif
