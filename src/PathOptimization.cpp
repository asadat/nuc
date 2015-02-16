#include "PathOptimization.h"

PathOptimization::PathOptimization(GNode *start_node):
    startNode(start_node)
{

}

PathOptimization::~PathOptimization()
{

}

bool PathOptimization::FindBestPath(GNode *goal, double costBudget, GNode::Path &p)
{
    static double best_reward = 0;

    readyNodes.push_back(startNode);
    GNode::Path *init_p = new GNode::Path();
    init_p->InitPath(NULL, startNode);

    startNode->AddBestPath(init_p);

    closedNodes.clear();

    while(!readyNodes.empty())
    {
        GNode* curNode = readyNodes.front();
        readyNodes.erase(readyNodes.begin());

        //printf("expanding node: %s #next: %d #paths: %d\n", curNode->label.c_str(), curNode->next.size(), curNode->bestPaths.size());

        for(unsigned int i=0; i < curNode->next.size(); ++i)
        {
            for(unsigned int j=0; j < curNode->bestPaths.size(); j++)
            {
                if(curNode->bestPaths[j]->pruned)
                    continue;

                double r = curNode->bestPaths[j]->NextReward(curNode->next[i]);
                double c = curNode->bestPaths[j]->NextCost(curNode->next[i]);

                if(!curNode->next[i]->ShouldBePruned(r, c, costBudget))
                {
                    GNode::Path * path = new GNode::Path();
                    path->InitPath(curNode->bestPaths[j], curNode->next[i]);
                    curNode->next[i]->AddBestPath(path);
                }
            }

            curNode->next[i]->visited_c +=1;
            if(curNode->next[i]->visited_c >= curNode->next[i]->prev.size())
            {
                readyNodes.push_back(curNode->next[i]);
            }
        }

        if(curNode == goal)
        {
            printf("Reached the goal. best reward reached %f\n", best_reward);
            goal->GetMaxRewardPath(p);
            return true;
        }
    }

    printf("No path is found.");
    return false;
}
