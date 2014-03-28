#ifndef _DEPTHFIRST_STRATEGY_
#define _DEPTHFIRST_STRATEGY_

#include "TraversalStrategy.h"
#include "CNode.h"

class DepthFirstStrategy: public TraversalStrategy
{
public:
    DepthFirstStrategy(CNode* root);
    ~DepthFirstStrategy();

    CNode* GetNextNode();

private:
    std::vector<CNode*> nodeStack;
};

#endif
