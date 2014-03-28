#ifndef _TRAVERSAL_STRATEGY_
#define _TRAVERSAL_STRATEGY_

#include "CNode.h"

class TraversalStrategy
{
public:
    virtual CNode* GetNextNode()=0;
    virtual void glDraw()=0;
};

#endif
