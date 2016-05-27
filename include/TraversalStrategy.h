#ifndef _TRAVERSAL_STRATEGY_
#define _TRAVERSAL_STRATEGY_

#include "CNode.h"
#include "map"

class TraversalStrategy
{
public:
    virtual ~TraversalStrategy(){}
    virtual CNode* GetNextNode()=0;
    virtual void glDraw()=0;
    virtual void hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey){}
};

#endif
