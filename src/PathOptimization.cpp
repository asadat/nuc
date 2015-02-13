#include "PathOptimization.h"

PathOptimization::PathOptimization(GNode *start_node):
    startNode(start_node)
{

}

PathOptimization::~PathOptimization()
{

}

bool PathOptimization::FindBestPath(GNode *goal, double costBudget, Path &p)
{
    readyNodes.push_back(startNode);
    Path *init_p = new Path(NULL, startNode);
    startNode->PruneOrAddBestPath(init_p, costBudget);

    closedNodes.clear();

    while(!readyNodes.empty())
    {
        GNode* curNode = readyNodes.front();
        readyNodes.erase(readyNodes.begin());

        for(unsigned int i=0; i < curNode->next.size(); ++i)
        {
            for(unsigned int j=0; j < curNode->bestPaths.size(); j++)
            {
                if(curNode->bestPaths[j]->pruned)
                    continue;

                Path * path = new Path(curNode->bestPaths[j], curNode->next[i]);
                curNode->next[i]->PruneOrAddBestPath(path, costBudget);
            }

            // check to see if the node is ready to be processed
            map<GNode*,int>::iterator it = openNodes.find(curNode->next[i]);
            bool moveToReady = false;

            if(it != openNodes.end())
            {
                it->second += 1;
                if(it->second >= curNode->next[i]->prev.size())
                {
                    moveToReady = true;
                    //readyNodes->push_back(curNode->next[i]);
                    openNodes.erase(it);
                }
            }
            else
            {
                if(curNode->next[i]->prev.size() == 1)
                {
                    moveToReady = true;
                    //readyNodes->push_back(curNode->next[i]);
                }
                else
                {
                    openNodes[curNode->next[i]] = 1;
                }
            }

            if(moveToReady && goal == curNode->next[i])
            {
                goal->GetMaxRewardPath(p);
                return true;
            }
            else if(moveToReady)
            {
                readyNodes.push_back(curNode->next[i]);
            }

        }

    }

    return false;
}
