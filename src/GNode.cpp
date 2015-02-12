#include "GNode.h"

GNode::GNode(CNode *node):
    cnode(node)
{

}

GNode::~GNode()
{

}

void GNode::AddNext(GNode *n)
{
    next.push_back(n);
}

void GNode::AddPrev(GNode *n)
{
    prev.push_back(n);
}
