#include "DepthFirstStrategy.h"

DepthFirstStrategy::DepthFirstStrategy(CNode *root)
{
    nodeStack.push_back(root);
}

DepthFirstStrategy::~DepthFirstStrategy()
{
    nodeStack.clear();
}

CNode* DepthFirstStrategy::GetNextNode()
{
    if(nodeStack.empty())
        return NULL;

    CNode * node = nodeStack.back();
    nodeStack.pop_back();

    for(int i=0; i<node->children.size(); i++)
    {
        CNode* child = node->children[i];
        if(child->IsNodeInteresting())
        {
            nodeStack.push_back(child);
        }
    }

    return node;
}
