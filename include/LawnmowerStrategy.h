#ifndef _LAWNMOWER_
#define _LAWNMOWER_

#include "TraversalStrategy.h"
#include "CNode.h"

class LawnmowerStrategy: public TraversalStrategy
{
public:
    LawnmowerStrategy(CNode* root, CNode* enterN=NULL, CNode* exitN=NULL);
    ~LawnmowerStrategy();

    void glDraw();
    CNode* GetNextNode();

private:
    std::vector<CNode*> nodeStack;
};

#endif
