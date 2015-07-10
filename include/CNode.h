#ifndef _CNODE_
#define _CNODE_
#include <vector>
#include <set>
#include "TooN/TooN.h"

typedef TooN::Vector<4> Rect;
class HilbertOptimization;
class HilbertStrategy;
class LawnmowerStrategy;
class DepthFirstStrategy;
class ShortCutStrategy;
class TestStrategy;
class NUC;
class GNode;
class TargetPolygon;

#define RAND(x,y) (x+((double)(rand()%1000)*0.001*(y-x)))


class CNode
{
public:
    CNode(Rect target_foot_print, bool populateChildren = true);
    ~CNode();

    // coverage related methods
    void PopulateChildren();

    // tree related methods
    CNode* CreateChildNode(Rect fp);

    bool IsNodeInteresting();
    void SetIsInteresting(bool interesting){isInterestingnessSet=true;isInteresting=interesting;}
    bool IsInterestingnessSet(){return isInterestingnessSet;}
    TooN::Vector<3> GetPos(){return pos;}
    TooN::Vector<3> GetMAVWaypoint();
    void glDraw();


    bool IsNeighbour(CNode*n);
    CNode * GetNeighbourLeaf(bool left, bool right, bool up, bool down, int inverse_depth=0);
    CNode* GetNearestNode(TooN::Vector<3> p);
    CNode* GetNearestLeaf(TooN::Vector<3> p, int revDepth=0);
    void GetNearestLeafAndParents(TooN::Vector<3> p, std::vector<CNode*> & list);
    void GetLeavesInRange(std::set<CNode*> &list, double range, TooN::Vector<3> center);

    Rect GetFootPrint(){return footPrint;}

    bool IsLeaf(){return children.empty();}

    int ComputeDepth(int & d);

    void propagateCoverage(double height);

    void GenerateObservationAndPropagate();
    void PropagateObservation(bool X);
    void UpdateProbability(double new_p_X);
    void RecomputeProbability();
    double RecomputePrior();
    void SetPrior(double p){p_X = p;}

    double CoverageReward();
    // if there is any descendant that is not visited or we
    // don't know if it is interesting or not
    bool NeedsVisitation();
    bool ChildrenNeedVisitation();
    bool ChildrenVisited();

    void GetUnvisitedFalseNegatives(int &n);
   // static float fov;
    //static int bf_sqrt;
    //static float minFootprintWidth;

    static int maxDepth;
    static double int_thr[20];
    static void PopulateInt_Thr(int maxdepth);

    static double Cost(CNode* from, CNode* to);

    static TooN::Vector<3> Rotation2D(TooN::Vector<3> v, double deg, TooN::Vector<2> c);

    static TooN::Vector<2> Rotation2D(TooN::Vector<2> v, double deg, TooN::Vector<2> c);

    static bool drawEdges;
    static bool drawCoverage;
    double p_X;
    double imgPrior;

    bool mav_wp_cached;
    TooN::Vector<3> mav_wp;

    CNode* GetParent(){return parent;}

    GNode * GetGNode(){return gnode;}
    void SetGNode(GNode* gn){gnode = gn;}

    void GenerateTargets(double prob_cutoff);
    void SetTreeVisited(bool visited);
    void SetAncestorVisited(bool av);
    void AddDependentPolygon(TargetPolygon* tp);
    void RemoveDependentPolygon(TargetPolygon *tp);

private:

    CNode* nextHilbertLeaf;
    GNode *gnode;
    std::vector<CNode*> children;
    std::vector<CNode*> ordered_children;

    //this is used for simulation only
    bool trueIsInteresting;
    bool PropagateInterestingness(Rect r);
    double InitializePrior(Rect r);
    void PropagateDepth();
    void PrintoutParams();

    double GetLocalPrior();

    bool VisitedInterestingDescendentExists();

    CNode *searchParentNode;
    CNode *parent;
    CNode *nds[8];
    bool neighbours_populated;

    bool visited;
    bool ancestor_visited;
    bool waiting;
    bool isInterestingnessSet;

    int grd_x, grd_y;
    int depth;
    Rect footPrint;
    TooN::Vector<3> pos;
    bool isInteresting;
    static double rootHeight;

    TooN::Vector<3> colorBasis;
    double coverage;

    bool extra_info;

    std::vector<TargetPolygon*> dependents;
    std::vector<TargetPolygon*> targets;

    int label;
    bool searchNode;

    friend class TargetPolygon;
    friend class SearchCoverageStrategy;
    friend class HilbertOptimization;
    friend class BudgetedStrategy;
    friend class HilbertStrategy;
    friend class LawnmowerStrategy;
    friend class DepthFirstStrategy;
    friend class ShortCutStrategy;
    friend class TestStrategy;
    friend class NUC;
    friend class GNode;
};


#endif
