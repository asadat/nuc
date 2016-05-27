#ifndef _BREADTHFIRST_STRATEGY_
#define _BREADTHFIRST_STRATEGY_

#include "TraversalStrategy.h"
#include "CNode.h"

class BreadthFirstStrategy: public TraversalStrategy
{
public:
    BreadthFirstStrategy(CNode* root);
    ~BreadthFirstStrategy();

    CNode* GetNextNode();
    void glDraw();

private:
    std::vector<CNode*> nodeStack;
    CNode * last;
};

#endif
