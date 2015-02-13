#ifndef _GNODE_
#define _GNODE_

#include <vector>
#include <string>
#include "TooN/TooN.h"

using namespace std;

class PathOptimization;
class CNode;
struct Path;

class GNode
{
public:
    GNode(CNode *node, std::string l="");
    GNode(TooN::Vector<3> pos, double reward);
    ~GNode();

    // constructing the graph
    void AddNext(GNode *n);
    void AddNext(CNode *n);

    void AddPrev(GNode *n);
    void AddPrev(CNode *n);


    // costs and rewards
    double NodeReward();
    double CostFrom(GNode* prevNode);
    double CostTo(GNode* nextNode);

    //path evaluation
    bool PruneOrAddBestPath(Path *p, double budget);
    bool GetMaxRewardPath(Path &p);

    std::string label;
    CNode * GetCNode(){return cnode;}

private:

    bool dummy;
    double dummy_reward;

    CNode * cnode;

    vector<GNode*> next;
    vector<GNode*> prev;

    vector<Path*> bestPaths; // best paths found to this node so far


    friend class PathOptimization;
};

struct Path
{
    Path()
    {
        reward =0;
        cost = 0;
        pruned = false;
    }

    Path(Path *p, GNode* n)
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

    void PrintOut()
    {
        printf("path length: %d\n", (int)path.size());
        for(unsigned int i=0; i<path.size(); i++)
        {
            printf("%s ->", path[i]->label.c_str());
        }

        printf("\n cost: %f  reward: %f \n", cost, reward);
    }

    vector<GNode*> path;
    double cost;
    double reward;

    bool pruned;
};

#endif
