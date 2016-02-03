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
    void SensingUpdate(TooN::Vector<3> pos);
private:
    Rect GetCourseSurveyFootprint();
    CNode * GetNode(int i, int j) const;
    CNode * GetSearchNode(int i, int j) const;
    void GenerateEnvironment(bool randomize=false);
    void CellularAutomataStep(vector<CNode*>&regions);
    bool InSearchGridBoundary(int i, int j) const;
    void SetupGrid(CNode* root);
    void GenerateLawnmower();
    inline bool SearchNodeExists() const {return !nodeStack.empty();}
    TooN::Vector<3> GetNextSearchPos();

    void OnReachedNode_GreedyPolicy(CNode * node, std::vector<TargetPolygon*> &newTargets, bool searchNode);
    void OnReachedNode_DelayedPolicy(CNode * node, std::vector<TargetPolygon*> &newTargets, bool searchNode);
    void OnReachedNode_DelayedGreedyPolicy(CNode * node, std::vector<TargetPolygon*> &newTargets, bool searchNode);

    void UpdateRemainingTime(CNode* node);

    double LawnmowerPlanValue(std::vector<TooN::Vector<3> > &lms, const CompoundTarget *ct, bool true_value = false);
    void PartiallyCoverTargets(vector<CompoundTarget*> &cts, const double budget, TooN::Vector<3> cur_pos, TooN::Vector<3> next_pos);
    void AddTargetsToComponentGenerators(vector<TargetPolygon*> &newTargets, CNode* node);
    void SetPolygonBoundaryFlags(TargetPolygon * plg, CNode* node, bool unvisitedBoundary, vector<CNode*> * bnodes=NULL);
    void SetCompoundTargetBoundaryFlags(CompoundTarget* ct);
    void EnqueueCompoundTarget(const CompoundTarget *ct);

    void FindSubCells(CNode* n);

    void FindClusters(bool incremental, std::vector<TargetPolygon*> & newTargets, CNode* ancester=NULL);

    void CleanupTargets();
    void CleanupComponents();

    void SetupCostsValues_NoExtensibleTarget(std::vector<CompoundTarget*> &cur_targets, std::vector<CompoundTarget*> &extensible_targets, CNode* cur_node);
    void SetupCostsValues_WithExtensibleTarget(std::vector<CompoundTarget*> &cur_targets, std::vector<CompoundTarget*> &extensible_targets, CNode* cur_node, bool delay);

    void SeparateCompoundTargets(const vector<CompoundTarget *> &all_targets, CNode* cur_search_node,
                                 vector<CompoundTarget*> &cur_targets, vector<CompoundTarget*> &extensible_targets);
    bool NeighboursNode(const CNode *n1, const CNode *n2) const;

    int ComputeCellularAutomataSteps(int fib, int clusters_count);

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

    bool drawFootprints;
    std::vector<TooN::Vector<3> > hi_res_waypoints;

    // delayed_greedy
    set<CompoundTarget*> targets2visit;
    double high_res_coverage;
    double high_res_coverage_true;
    std::vector<TooN::Vector<3> > turningLocations;

    long int used_seed;
};

#endif
