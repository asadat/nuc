#ifndef _GNODE_
#define _GNODE_

#include <vector>
#include <map>
#include <string>
#include "TooN/TooN.h"

using namespace std;

class PathOptimization;
class CNode;
struct Path;

class GNode
{
public:

    class Path
    {
    public:
        Path();
        //Path(Path *p, GNode* n);
        void InitPath(Path *p, GNode* n);
        void PrintOut();

        double NextReward(GNode *n);
        double NextCost(GNode* n);

        vector<GNode*> path;
        double cost;
        double reward;

        bool pruned;
    };

    GNode(CNode *node, std::string l="");
    GNode(TooN::Vector<3> pos, double reward);
    ~GNode();

    // constructing the graph
    void AddNext(GNode *n);
    void AddNext(CNode *n);

    void AddPrev(GNode *n);
    void AddPrev(CNode *n);


    // costs and rewards
    double NodeReward();
    double CostFrom(GNode* prevNode);
    double CostTo(GNode* nextNode);

    //path evaluation
    int AddBestPath(Path *p);
    bool ShouldBePruned(double r, double c, double budget);
    bool GetMaxRewardPath(Path &p);

    std::string label;
    CNode * GetCNode(){return cnode;}

    void glDraw();

    int visited_c;

private:

    bool dummy;
    double dummy_reward;

    bool leaf;

    CNode * cnode;

    vector<GNode*> next;
    vector<GNode*> prev;

    vector<GNode::Path*> bestPaths; // best paths found to this node so far

    map<GNode*,double> costTo;

    friend class HilbertOptimization;
    friend class PathOptimization;
};



#endif
