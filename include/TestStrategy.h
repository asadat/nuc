#ifndef _TEST_STRATEGY_
#define _TEST_STRATEGY_

#include "TraversalStrategy.h"
#include "CNode.h"

class TestStrategy: public TraversalStrategy
{
public:
    TestStrategy(CNode* root);
    ~TestStrategy();

    void glDraw();
    CNode* GetNextNode();

private:
    std::vector<CNode*> nodeStack;
};

#endif
