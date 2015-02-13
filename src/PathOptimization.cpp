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

        //printf("expanding node: %s #next: %d #paths: %d\n", curNode->label.c_str(), curNode->next.size(), curNode->bestPaths.size());

        for(unsigned int i=0; i < curNode->next.size(); ++i)
        {

            for(unsigned int j=0; j < curNode->bestPaths.size(); j++)
            {
                if(curNode->bestPaths[j]->pruned)
                    continue;

                Path * path = new Path(curNode->bestPaths[j], curNode->next[i]);
                if(!curNode->next[i]->PruneOrAddBestPath(path, costBudget))
                {
                    delete path;
                }
            }

            // check to see if the node is ready to be processed
            map<GNode*,int>::iterator it = openNodes.find(curNode->next[i]);
            bool moveToReady = false;

            if(it != openNodes.end())
            {
                it->second += 1;
                //openNodes[curNode->next[i]] += 1;

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

            if(moveToReady)
            {
                readyNodes.push_back(curNode->next[i]);
            }

        }

        if(curNode == goal)
        {
            printf("Reached the goal.\n");
            goal->GetMaxRewardPath(p);
            return true;
        }
    }

    printf("No path is found.");
    return false;
}
