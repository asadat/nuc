#ifndef _CNODE_
#define _CNODE_
#include <vector>
#include "TooN/TooN.h"

typedef TooN::Vector<4> Rect;
class HilbertStrategy;
class LawnmowerStrategy;
class DepthFirstStrategy;
class ShortCutStrategy;
class TestStrategy;
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
    void SetIsInteresting(bool interesting){isInterestingnessSet=true;isInteresting=interesting;}
    bool IsInterestingnessSet(){return isInterestingnessSet;}
    TooN::Vector<3> GetPos(){return pos;}
    TooN::Vector<3> GetMAVWaypoint();
    void glDraw();


    CNode* GetNearestNode(TooN::Vector<3> p);
    CNode* GetNearestLeaf(TooN::Vector<3> p);
    void GetNearestLeafAndParents(TooN::Vector<3> p, std::vector<CNode*> & list, int atDepth);

    Rect GetFootPrint(){return footPrint;}

    bool IsLeaf(){return children.empty();}

    void propagateCoverage(double height);

    // if there is any descendant that is not visited or we
    // don't know if it is interesting or not
    bool NeedsVisitation();
    bool ChildrenNeedVisitation();
   // static float fov;
    //static int bf_sqrt;
    //static float minFootprintWidth;

    static TooN::Vector<3> Rotation2D(TooN::Vector<3> v, double deg, TooN::Vector<2> c);

    static TooN::Vector<2> Rotation2D(TooN::Vector<2> v, double deg, TooN::Vector<2> c);

    static bool drawEdges;
    static bool drawCoverage;

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
    bool isInterestingnessSet;

    int grd_x, grd_y;
    int depth;
    Rect footPrint;
    TooN::Vector<3> pos;
    bool isInteresting;
    static double rootHeight;

    double coverage;

    friend class HilbertStrategy;
    friend class LawnmowerStrategy;
    friend class DepthFirstStrategy;
    friend class ShortCutStrategy;
    friend class TestStrategy;
    friend class NUC;
};


#endif
