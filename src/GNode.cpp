#include "GNode.h"
#include "CNode.h"
#include <GL/glut.h>
#include "ros/ros.h"

GNode::Path::Path()
{
    reward =0;
    cost = 0;
    pruned = false;
}

void GNode::Path::InitPath(Path *p, GNode* n)
{
    if(p)
    {
        copy(p->path.begin(),p->path.end(), std::back_inserter(path));
        reward = p->reward + n->NodeReward();
        cost = p->cost + path.back()->CostTo(n);
        pruned = false;
    }
    else
    {
        reward = n->NodeReward();
        cost = 0;
        pruned = false;
    }

    path.push_back(n);
}

void GNode::Path::ReplaceNode(GNode *n, vector<CNode *> &nds)
{
    for(unsigned int i=0; i < path.size(); i++)
    {
        if(path[i] == n)
        {
            path.erase(path.begin()+i);

            for(unsigned int j=0; j<nds.size(); j++)
            {
                path.insert(path.begin()+i+j, nds[j]->GetGNode());
            }
        }
    }

    this->UpdateRewardCost();
}

double GNode::Path::ReplaceNodeReward(GNode *n, vector<CNode *> &nds, double budget) const
{
    double rw = 0;
    double cst = 0;
    for(unsigned int i=0; i < path.size()-1; i++)
    {
        if(path[i+1] == n)
        {
            for(unsigned int j=0; j<nds.size(); j++)
            {
                rw += nds[j]->CoverageReward();
                cst += path[i]->CostTo(nds[j]->GetGNode());
            }
        }
        else  if(path[i] == n)
        {
            rw += path[i+1]->NodeReward();
            cst += nds.back()->GetGNode()->CostTo(path[i+1]);
        }
        else
        {
            rw += path[i+1]->NodeReward();
            cst += path[i]->CostTo(path[i+1]);
        }

        if(cst > budget)
        {
            rw = -1;
            break;
        }
    }

    return  rw;
}

void GNode::Path::UpdateRewardCost()
{
    if(path.size()<2)
        return ;

    reward = path[0]->NodeReward();
    cost = 0;

    for(unsigned int i=0; i< path.size()-1; i++)
    {
        reward += path[i+1]->NodeReward();
        cost += path[i]->CostTo(path[i+1]);
    }
}

void GNode::Path::PrintOut()
{
    printf("path length: %d\n", (int)path.size());
    for(unsigned int i=0; i<path.size(); i++)
    {
        printf("%s ->", path[i]->label.c_str());
    }

    printf("\n cost: %f  reward: %f \n", cost, reward);
}

double GNode::Path::NextReward(GNode *n)
{
    return reward + n->NodeReward();
}

double GNode::Path::NextCost(GNode *n)
{
    return cost + path.back()->CostTo(n);
}

GNode::GNode(CNode *node, string l):
    cnode(node),
    label(l)
{
    cnode->SetGNode(this);
    visited_c = 0;
    dummy = false;
    visited_c = 0;
    dummy_reward = 0;
    leaf = false;
    greedy_count = 0;
    maxRewardToGoal = 0;
    minPathToGoalCost = 0;
    minPathToGoalReward = 0;

}

GNode::GNode(TooN::Vector<3> pos, double reward)
{
    visited_c = 0;
    dummy = true;
    dummy_reward = reward;
    cnode = new CNode(TooN::makeVector(0,0,1,1));
    cnode->SetGNode(this);
    cnode->pos = pos;
    leaf = false;
    greedy_count = 0;
    maxRewardToGoal = 0;
    minPathToGoalCost = 0;
    minPathToGoalReward = 0;

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
    if(n)
    {
        if(!n->GetGNode())
        {
            GNode * g = new GNode(n);
        }

        AddNext(n->GetGNode());
    }
}

void GNode::AddPrev(GNode *n)
{
    prev.push_back(n);
}

void GNode::AddPrev(CNode *n)
{
    if(n)
    {
        if(!n->GetGNode())
        {
            GNode * g = new GNode(n);
        }
        AddPrev(n->GetGNode());
    }
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
    if(dummy || prevNode->dummy)
        return 0;

    return CNode::Cost(prevNode->cnode, cnode);
}

double GNode::CostTo(GNode *nextNode)
{
    if(dummy || nextNode->dummy)
        return 0;

    map<GNode*,double>::iterator it = costTo.find(nextNode);

    if( it != costTo.end())
    {
        return it->second;
    }
    else
    {
        double c = CNode::Cost(cnode, nextNode->cnode);
        costTo[nextNode] = c;
        return c;
    }
}

bool GNode::GetMaxRewardPath(GNode::Path &p)
{
    printf("#Paths: %d\n", (int)bestPaths.size());
    int idx = -1;
    double maxReward = 0;

    for(int i=0; i<bestPaths.size(); i++)
    {
        //printf("PATH_REWARD: %f \n",bestPaths[i]->reward);

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

bool GNode::ShouldBePruned(double r, double c, double budget, double greedy_reward, double leastRewardFound)
{
    static int nnn=0;
    if(c + minPathToGoalCost > budget)
    {
        return true;
    }

    for(unsigned int i=0; i < bestPaths.size(); i++)
    {
        if(bestPaths[i]->pruned)
            continue;


        if(cnode->nextHilbertLeaf)
        {
            if(leastRewardFound > r + cnode->nextHilbertLeaf->GetGNode()->maxRewardToGoal)
            {
                nnn++;
                ROS_INFO_THROTTLE_NAMED(1,"UBOUND","UBound %d: %f CUR: %f %f",nnn, leastRewardFound, r, cnode->nextHilbertLeaf->GetGNode()->maxRewardToGoal);

                return true;
            }
        }
        if(cnode->nextHilbertLeaf != NULL)
        {
           // ROS_INFO_THROTTLE_NAMED(0.5,"PRUNE","Checking by greedy bound.");
            if(greedy_reward >= r + cnode->nextHilbertLeaf->GetGNode()->maxRewardToGoal )
            {
                //ROS_INFO("PRUNED by greedy bound.");
                return true;
            }
        }

        if((bestPaths[i]->cost <= c && bestPaths[i]->reward >= r))
        {
            return true;
        }

        if((bestPaths[i]->cost > c && bestPaths[i]->reward < r))
        {
            bestPaths[i]->pruned = true;
            bestPaths.erase(bestPaths.begin()+i);
            i--;
        }
    }

    return false;
}

int GNode::AddBestPath(Path *p)
{
    bestPaths.push_back(p);
    return bestPaths.size()-1;
}

void GNode::glDraw()
{

    //glColor3f(1,0.5,0.0);

    for(unsigned int i=0; i<next.size();i++)
    {
        TooN::Vector<3> p1 = cnode->GetPos();
        TooN::Vector<3> p2 = next[i]->cnode->GetPos();


        glColor3f(1,0,0);
        glVertex3f(p1[0],p1[1],p1[2]);
        glColor3f(0,1,0);
        glVertex3f(p2[0],p2[1],p2[2]);

        next[i]->glDraw();
    }

}
