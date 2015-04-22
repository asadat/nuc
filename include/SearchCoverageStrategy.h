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

    std::vector<CNode*> nodeStack;
    std::vector<CNode*> visitedNodes;
    std::vector<CNode*> grid;

    double cutoff_prob;
    int cluster_n;
    std::vector<TooN::Vector<3> > c;
    std::multimap<int, CNode*> clusters;
    std::vector<std::vector<CNode*> * > hulls;

    CNode* tree;
    int s;
};

#endif
