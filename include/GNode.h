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

private:
    CNode * cnode;

    vector<GNode*> next;
    vector<GNode*> prev;

};

#endif
