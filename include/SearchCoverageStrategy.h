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

    void SetPolygonBoundaryFlags(TargetPolygon * plg, CNode* node, bool unvisitedBoundary, vector<CNode*> * bnodes=NULL);
    void SetCompoundTargetBoundaryFlags(CompoundTarget* ct);

    void FindSubCells(CNode* n);

    void FindClusters(bool incremental, std::vector<TargetPolygon*> & newTargets);

    void CleanupTargets();
    void CleanupComponents();

    void GetNearestStartCellAndCost(std::vector<CompoundTarget*> &cmpn, CNode* cur_node);
    void SetupCostsValuesCase_1(std::vector<CompoundTarget*> &cur_targets, std::vector<CompoundTarget*> &extensible_targets, CNode* cur_node);

    void SeparateCompoundTargets(vector<CompoundTarget*> &all_targets, CNode* cur_search_node,
                                 vector<CompoundTarget*> &cur_targets, vector<CompoundTarget*> &extensible_targets);
    bool NeighboursNode(CNode* n1, CNode* n2);
    void ExtractPlanFromTargets(set<CompoundTarget*> final_targets, CNode* cur_node);

    std::vector<CNode*> nodeStack;
    std::vector<CNode*> visitedNodes;
    std::vector<CNode*> grid;
    std::vector<CNode*> search_grid;
    std::vector<TooN::Vector<3> > target_lms;

    GraphComponents gc;
    vector<CompoundTarget*> components;
    vector<CompoundTarget*> integrated_components;


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

    // delayed_greedy
    //vector<CNode*> start_nodes;
    //vector<double> target_costs;
    //vector<double> target_values;
    set<CompoundTarget*> targets2visit;
};

#endif
