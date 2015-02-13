#include "GNode.h"
#include "CNode.h"

GNode::GNode(CNode *node, string l):
    cnode(node),
    label(l)
{
    cnode->SetGNode(this);
}

GNode::GNode(TooN::Vector<3> pos, double reward)
{
    dummy = true;
    dummy_reward = reward;
    cnode = new CNode(TooN::makeVector(0,0,1,1));
    cnode->SetGNode(this);
    cnode->pos = pos;
}

GNode::~GNode()
{
    while(!bestPaths.empty())
    {
        delete bestPaths.back();
        bestPaths.pop_back();
    }

    if(dummy)
    {
        delete cnode;
    }
}

void GNode::AddNext(GNode *n)
{
    next.push_back(n);
    n->AddPrev(this);
}

void GNode::AddNext(CNode *n)
{
    if(!n->GetGNode())
    {
        GNode * g = new GNode(n);
    }

    AddNext(n->GetGNode());
}

void GNode::AddPrev(GNode *n)
{
    prev.push_back(n);
}

void GNode::AddPrev(CNode *n)
{
    if(!n->GetGNode())
    {
        GNode * g = new GNode(n);
    }
    AddPrev(n->GetGNode());
    //AddPrev(g);
}

double GNode::NodeReward()
{
    if(dummy)
    {
        return dummy_reward;
    }
    else
    {
        return cnode->CoverageReward();
    }
}

double GNode::CostFrom(GNode *prevNode)
{
    return CNode::Cost(prevNode->cnode, cnode);
}

double GNode::CostTo(GNode *nextNode)
{
    return CNode::Cost(cnode, nextNode->cnode);
}

bool GNode::GetMaxRewardPath(Path &p)
{
    printf("#Paths: %d\n", (int)bestPaths.size());
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
        //p->pruned = true;
        //bestPaths.push_back(p);
        return false;
    }

    for(unsigned int i=0; i < bestPaths.size(); i++)
    {
        if(bestPaths[i]->pruned)
            continue;

        if((bestPaths[i]->cost < p->cost && bestPaths[i]->reward > p->reward))
        {
            //p->pruned = true;
            //bestPaths.push_back(p);
            return false;
        }

        if((bestPaths[i]->cost > p->cost && bestPaths[i]->reward < p->reward))
        {
            //bestPaths[i]->pruned = true;
            bestPaths.erase(bestPaths.begin()+i);
            i--;
        }
    }

    bestPaths.push_back(p);
    return true;
}
