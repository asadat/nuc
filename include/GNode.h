#ifndef _GNODE_
#define _GNODE_

#include "CNode.h"
#include <vector>

using namespace std;

class GNode
{
public:
    GNode(CNode *node);
    ~GNode();

    void AddNext(GNode *n);
    void AddPrev(GNode *n);
    double NodeReward();

    double CostFrom(GNode* prevNode);
    double CostTo(GNode* nextNode);

private:
    CNode * cnode;

    vector<GNode*> next;
    vector<GNode*> prev;

};

#endif
