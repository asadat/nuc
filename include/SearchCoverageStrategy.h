#ifndef _SEARCHCOVERAGE_STRATEGY_H
#define _SEARCHCOVERAGE_STRATEGY_H
#include "TraversalStrategy.h"
#include "CNode.h"
#include "GraphComponents.h"
#include "Knapsack.h"

class TargetPolygon;

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
    CNode * GetSearchNode(int i, int j);
    bool InSearchGridBoundary(int i, int j);
    void SetupGrid(CNode* root);
    void GenerateLawnmower();

    void OnReachedNode_GreedyPolicy(CNode * node, std::vector<TargetPolygon*> &newTargets, bool searchNode);
    void OnReachedNode_DelayedPolicy(CNode * node, std::vector<TargetPolygon*> &newTargets, bool searchNode);
    void OnReachedNode_DelayedGreedyPolicy(CNode * node, std::vector<TargetPolygon*> &newTargets, bool searchNode);
    void UpdateRemainingTime(CNode* node);

    void SetPolygonBoundaryFlags(TargetPolygon * plg, CNode* node);

    void FindSubCells(CNode* n);

    void FindClusters(bool incremental, std::vector<TargetPolygon*> & newTargets);

    void CleanupTargets();

    std::vector<CNode*> nodeStack;
    std::vector<CNode*> visitedNodes;
    std::vector<CNode*> grid;
    std::vector<CNode*> search_grid;
    std::vector<TooN::Vector<3> > target_lms;

    GraphComponents gc;
    vector<vector<TargetPolygon*> *> components;
    vector<vector<TargetPolygon*> *> integrated_components;


    double cutoff_prob;
    int cluster_n;
    std::vector<TooN::Vector<3> > c;
    std::multimap<int, CNode*> clusters;
    std::vector<TargetPolygon*> targets;

    CNode* dummy;

    CNode* tree;
    int s;
    int search_cell_size;
    double remaining_time;
    TooN::Vector<3> startPos;
    TooN::Vector<3> prevGoal;

};

#endif
