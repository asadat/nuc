#ifndef _SEARCHCOVERAGE_STRATEGY_H
#define _SEARCHCOVERAGE_STRATEGY_H
#include "TraversalStrategy.h"
#include "CNode.h"

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
    void SetupGrid(CNode* root);
    void GenerateLawnmower();
    double GetPlanExecutionTime(std::vector<TooN::Vector<3> > & wps, TooN::Vector<3> curpos, TooN::Vector<3> endpos, bool initalTurn, bool endTurn);
    double GetPlanExecutionTime(std::vector<CNode*> & wps, TooN::Vector<3> curpospos, TooN::Vector<3> endpos, bool initalTurn, bool endTurn);
    double GetPlanExecutionTime(std::vector<TooN::Vector<3> > & wps, bool ignoreFirstSegment, bool ignoreLastSegment);

    void OnReachedNode_GreedyPolicy(CNode * node, std::vector<TargetPolygon*> &newTargets, bool searchNode, int newTargetIdxBegin);
    void OnReachedNode_DelayedPolicy(CNode * node, std::vector<TargetPolygon*> &newTargets, bool searchNode, int newTargetIdxBegin);
    void OnReachedNode_DelayedGreedyPolicy(CNode * node, std::vector<TargetPolygon*> &newTargets, bool searchNode, int newTargetIdxBegin);
    void UpdateRemainingTime(CNode* node);

    void SimplifyTargetSet(std::vector<TargetPolygon*> &targets);
    void FindSubCells(CNode* n);

    void FindClusters(bool incremental, std::vector<TargetPolygon*> & newTargets);

    void CleanupTargets();

    std::vector<CNode*> nodeStack;
    std::vector<CNode*> visitedNodes;
    std::vector<CNode*> grid;
    std::vector<TooN::Vector<3> > target_lms;

    double cutoff_prob;
    int cluster_n;
    std::vector<TooN::Vector<3> > c;
    std::multimap<int, CNode*> clusters;
    std::vector<TargetPolygon*> targets;

    CNode* dummy;

    CNode* tree;
    int s;
    double remaining_time;
    TooN::Vector<3> startPos;
    TooN::Vector<3> prevGoal;

};

#endif
