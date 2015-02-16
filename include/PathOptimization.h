#ifndef _PATH_OPTIMIZATION_
#define _PATH_OPTIMIZATION_

#include "GNode.h"
#include <map>
#include <set>

using namespace std;
class PathOptimization
{
public:
    PathOptimization(GNode* start_node);
    ~PathOptimization();

    bool FindBestPath(GNode* goal, double costBudget, GNode::Path & p);

private:

    set<GNode*> closedNodes; // already processed
    vector<GNode*> readyNodes; // can be expanded forward
    map<GNode*, int> openNodes; // still waiting for the rest of the paths to them

    GNode* startNode;
};

#endif
