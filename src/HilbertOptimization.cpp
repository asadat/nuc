#include "HilbertOptimization.h"

HilbertOptimization::HilbertOptimization(CNode *root):
    HilbertStrategy(root)
{

    GNode* g[6];
    g[0] = new GNode(NULL, "1");
    g[1] = new GNode(NULL, "2");
    g[2] = new GNode(NULL, "3");
    g[3] = new GNode(NULL, "4");
    g[4] = new GNode(NULL, "5");
    g[5] = new GNode(NULL, "6");

    g[0]->AddNext(g[1]);
    g[0]->AddNext(g[2]);
    g[0]->AddNext(g[3]);

    g[1]->AddNext(g[5]);
    g[1]->AddNext(g[4]);

    g[2]->AddNext(g[1]);
    g[2]->AddNext(g[4]);
    g[2]->AddNext(g[3]);

    g[3]->AddNext(g[4]);
    g[3]->AddNext(g[5]);

    g[4]->AddNext(g[5]);

    optimizer = new PathOptimization(g[0]);
    Path p;
    optimizer->FindBestPath(g[5], 10.0, p);
    p.PrintOut();

}

HilbertOptimization::~HilbertOptimization()
{

}

CNode* HilbertOptimization::GetNextNode()
{
    return NULL;
}

