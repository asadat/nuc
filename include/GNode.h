#ifndef _GNODE_
#define _GNODE_

#include "CNode.h"
#include <vector>

using namespace std;

class PathOptimization;

struct Path
{
    vector<GNode*> path;
    double cost;
    double reward;

};


class GNode
{
public:
    GNode(CNode *node);
    ~GNode();

    // constructing the graph
    void AddNext(GNode *n);
    void AddPrev(GNode *n);


    // costs and rewards
    double NodeReward();
    double CostFrom(GNode* prevNode);
    double CostTo(GNode* nextNode);

private:
    CNode * cnode;

    vector<GNode*> next;
    vector<GNode*> prev;

    vector<Path*> bestPaths; // best paths found to this node so far

    friend class PathOptimization;
};

#endif
