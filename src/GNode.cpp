#include "GNode.h"
#include "CNode.h"

GNode::GNode(CNode *node, string l):
    cnode(node),
    label(l)
{

}

GNode::~GNode()
{
    while(!bestPaths.empty())
    {
        delete bestPaths.back();
        bestPaths.pop_back();
    }
}

void GNode::AddNext(GNode *n)
{
    next.push_back(n);
    n->AddPrev(this);
}

void GNode::AddPrev(GNode *n)
{
    prev.push_back(n);
    n->AddNext(this);
}

double GNode::NodeReward()
{
    return 1.0;
    //return cnode->CoverageReward();
}

double GNode::CostFrom(GNode *prevNode)
{
    return 1.0;
    //return CNode::Cost(prevNode->cnode, cnode);
}

double GNode::CostTo(GNode *nextNode)
{
    return 1.0;
    //return CNode::Cost(cnode, nextNode->cnode);
}

bool GNode::GetMaxRewardPath(Path &p)
{
    int idx = -1;
    double maxReward = 0;

    for(unsigned int i=0; i<bestPaths.size(); i++)
    {
        if(!bestPaths[i]->pruned && bestPaths[i]->reward > maxReward)
        {
            idx = i;
            maxReward = bestPaths[i]->reward;
        }
    }

    if(idx > -1)
    {
        copy(bestPaths[idx]->path.begin(), bestPaths[idx]->path.end(), std::back_inserter(p.path));
        p.cost = bestPaths[idx]->cost;
        p.reward = bestPaths[idx]->reward;

        return true;
    }
    else
    {
        return false;
    }
}

bool GNode::PruneOrAddBestPath(Path *p, double budget)
{
    if(p->cost > budget)
    {
        p->pruned = true;
        bestPaths.push_back(p);
        return false;
    }

    for(unsigned int i=0; i < bestPaths.size(); i++)
    {
        if(bestPaths[i]->pruned)
            continue;

        if((bestPaths[i]->cost < p->cost && bestPaths[i]->reward > p->reward))
        {
            p->pruned = true;
            bestPaths.push_back(p);
            return true;
        }

        if((bestPaths[i]->cost > p->cost && bestPaths[i]->reward < p->reward))
        {
            bestPaths[i]->pruned = true;
        }
    }

    bestPaths.push_back(p);
    return true;
}
