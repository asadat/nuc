#ifndef _SEARCHCOVERAGE_STRATEGY_H
#define _SEARCHCOVERAGE_STRATEGY_H
#include "TraversalStrategy.h"
#include "CNode.h"



class SearchCoverageStrategy: public TraversalStrategy
{
public:
    SearchCoverageStrategy(CNode* root);
    ~SearchCoverageStrategy();

    void glDraw();
    CNode* GetNextNode();
    void hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey);
    void ReachedNode(CNode *node);

private:
    CNode * GetNode(int i, int j);
    void SetupGrid(CNode* root);
    void GenerateLawnmower();

    void FindClusters();
    void ConvexHull(int label, std::vector<CNode*>& ch);
    void FindConvexHulls();

    //convex polygon lawnmower
    double pointToLineDist(TooN::Vector<3> p1, TooN::Vector<3> p2, TooN::Vector<3> x);
    double pointToLineSignedDist(TooN::Vector<3> p1, TooN::Vector<3> p2, TooN::Vector<3> x);
    bool GetLineSegmentIntersection(TooN::Vector<3> p1, TooN::Vector<3> p2, TooN::Vector<3> p3, TooN::Vector<3> p4, TooN::Vector<3> &intersection_p);

    std::pair<int, int> BaseEdge(std::vector<CNode*> & ch, double & height);
    void PlanLawnmovers();
    void PlanLawnmower(std::vector<CNode*> * ch, int baseStart_idx, int baseEnd_idx, double height, std::vector<TooN::Vector<3> > * lm);

    std::vector<CNode*> nodeStack;
    std::vector<CNode*> visitedNodes;
    std::vector<CNode*> grid;
    double cellW;

    double cutoff_prob;
    int cluster_n;
    std::vector<TooN::Vector<3> > c;
    std::multimap<int, CNode*> clusters;
    std::vector<std::vector<CNode*> * > hulls;
    std::vector<std::pair<int,int> > baseEdges;
    std::vector<double> chHeights;
    std::map<int, std::vector<TooN::Vector<3> > * > lawnmovers;


    CNode* tree;
    int s;
};

#endif
