#ifndef _BUDGETED_STRATEGY_H
#define _BUDGETED_STRATEGY_H
#include "TraversalStrategy.h"
#include "CNode.h"
#include <map>

class BudgetedStrategy: public TraversalStrategy
{
public:
    BudgetedStrategy(CNode* root);
    ~BudgetedStrategy();

    void glDraw();
    CNode* GetNextNode();
    void hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey);

private:
    void UpdateGridCutoff(CNode* n);
    CNode * GetNode(int i, int j);
    void FindClusters();
    void SetupGrid(CNode* root);
    void ConvexHull(int label, std::vector<CNode*>& ch);
    void FindConvexHulls();

    std::vector<CNode*> nodeStack;
    std::vector<CNode*> grid;
    double cutoff_prob;
    CNode* tree;
    int s;
    bool drawDubins;

    int cluster_n;
    std::vector<TooN::Vector<3> > c;
    std::multimap<int, CNode*> clusters;
    std::vector<TooN::Vector<2> > dp;
    std::vector<std::vector<CNode*> * > hulls;

    //std::vector<CNode*> hull;
};

#endif
