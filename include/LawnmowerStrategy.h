#ifndef _LAWNMOWER_
#define _LAWNMOWER_

#include "TraversalStrategy.h"
#include "CNode.h"

class LawnmowerStrategy: public LawnmowerStrategy
{
public:
    LawnmowerStrategy(CNode* root);
    ~LawnmowerStrategy();

    CNode* GetNextNode();

private:
    std::vector<CNode*> nodeStack;
};

#endif
