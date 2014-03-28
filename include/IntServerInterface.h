#ifndef _INTSERVERINTERFACE_
#define _INTSERVERINTERFACE_

#include "CNode.h"

class IntServerInterface
{
public:
    virtual bool IsNodeInteresting(const CNode* nnode)=0;
};

#endif
