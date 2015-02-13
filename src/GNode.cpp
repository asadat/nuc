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

double GNode::NodeReward()
{
    return cnode->CoverageReward();
}

double GNode::CostFrom(GNode *prevNode)
{
    return CNode::Cost(prevNode->cnode, cnode);
}

double GNode::CostTo(GNode *nextNode)
{
    return CNode::Cost(cnode, nextNode->cnode);
}
