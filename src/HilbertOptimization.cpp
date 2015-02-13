#include "HilbertOptimization.h"

HilbertOptimization::HilbertOptimization(CNode *root, TooN::Vector<3> init_pos, TooN::Vector<3> end_pos):
    HilbertStrategy(root)
{


    startNode = new GNode(init_pos, 0);
    endNode = new GNode(end_pos, 0);

    //generate forward links
    for(unsigned int i=0; i < lastDepth; i++)
    {
        startNode->AddNext(hilbert[i][0]);
        for(unsigned int j=0; j < hilbert[i].size()-1; j++)
        {
            hilbert[i][j]->GetGNode()->AddNext(hilbert[i][j+1]);
        }

        hilbert[i].back()->GetGNode()->AddNext(endNode);
    }

    optimizer = new PathOptimization(startNode);
    optimizer->FindBestPath(endNode, 10000.0, p);
    p.PrintOut();

}

HilbertOptimization::~HilbertOptimization()
{

}

CNode* HilbertOptimization::GetNextNode()
{
    static int i=0;

    if(i >= p.path.size()-1)
    {
        return NULL;
    }
    else
    {
        i++;
        return p.path[i]->GetCNode();
    }

}

