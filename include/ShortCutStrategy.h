#ifndef _SHORTCUT_STRATEGY_
#define _SHORTCUT_STRATEGY_

#include "TraversalStrategy.h"
#include "CNode.h"

class ShortCutStrategy: public TraversalStrategy
{
public:
    ShortCutStrategy(CNode* root);
    ~ShortCutStrategy();

    CNode* GetNextNode();
    void glDraw();

private:
    std::vector<CNode*> nodeStack;
    CNode * last;
};

#endif
