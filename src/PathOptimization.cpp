#include "PathOptimization.h"
#include "ros/ros.h"

PathOptimization::PathOptimization(GNode *start_node):
    startNode(start_node)
{
    leastFeasibleRewardSoFar = 0;
}

PathOptimization::~PathOptimization()
{

}

bool PathOptimization::FindBestPath(GNode *goal, double costBudget, double greedy_reward, GNode::Path &p)
{
    static double best_reward = 0;

    readyNodes.push_back(startNode);
    GNode::Path *init_p = new GNode::Path();
    init_p->InitPath(NULL, startNode);

    startNode->AddBestPath(init_p);

    closedNodes.clear();

    while(!readyNodes.empty())
    {
        bool check_for_ready = true;

        GNode* curNode = readyNodes.front();
        readyNodes.erase(readyNodes.begin());

        //printf("#%d   expanding node: %s #next: %d #paths: %d\n", readyNodes.size(), curNode->label.c_str(), curNode->next.size(), curNode->bestPaths.size());

        for(unsigned int i=0; i < curNode->next.size(); ++i)
        {
            for(unsigned int j=0; j < curNode->bestPaths.size(); j++)
            {
                if(curNode->bestPaths[j]->pruned)
                    continue;

                // check if this is in the leaf level and
                // we can easily add the next 3 nodes to the path
                if(curNode->leaf & curNode->next.size()==1 )
                {
                    int pidx = j;
                    GNode * gn = curNode;
                    while(gn->next.size() == 1)
                    {
                        GNode::Path * sc = new GNode::Path();
                        sc->InitPath(gn->bestPaths[pidx], gn->next[0]);

                        pidx = gn->next[0]->AddBestPath(sc);
                        gn = gn->next[0];
                    }

                    // add the end of mini path to the ready node once
                    if(check_for_ready)
                    {
                        if(gn->bestPaths[pidx]->cost > costBudget)
                            gn->bestPaths[pidx]->pruned = true;

                        readyNodes.push_back(gn);

                        check_for_ready = false;
                    }

                    continue;
                }

                double r = curNode->bestPaths[j]->NextReward(curNode->next[i]);
                double c = curNode->bestPaths[j]->NextCost(curNode->next[i]);

                if(!curNode->next[i]->ShouldBePruned(r, c, costBudget, greedy_reward, leastFeasibleRewardSoFar))
                {
                    GNode::Path * path = new GNode::Path();
                    path->InitPath(curNode->bestPaths[j], curNode->next[i]);
                    curNode->next[i]->AddBestPath(path);

                    if(path->cost + path->path.back()->minPathToGoalCost <= costBudget
                            && path->reward + path->path.back()->minPathToGoalReward > leastFeasibleRewardSoFar)
                    {
                        leastFeasibleRewardSoFar = path->reward + path->path.back()->minPathToGoalReward;
                       // ROS_INFO("Least feasible reward: %f %f\n", path->reward, leastFeasibleRewardSoFar);

                    }
                }
            }

            if(check_for_ready)
            {
                curNode->next[i]->visited_c +=1;
                if(curNode->next[i]->visited_c == curNode->next[i]->prev.size())
                {
                    readyNodes.push_back(curNode->next[i]);
                }
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
