#ifndef _TEST_STRATEGY_
#define _TEST_STRATEGY_

#include "TraversalStrategy.h"
#include "CNode.h"
#include "dubins.h"


class TestStrategy: public TraversalStrategy
{
public:
    TestStrategy(CNode* root);
    ~TestStrategy();

    void glDraw();
    CNode* GetNextNode();
    //int dubin_cb(double q[3], double x, void* user_data);
private:
    std::vector<CNode*> nodeStack;
};

#endif
