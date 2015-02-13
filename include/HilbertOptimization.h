#ifndef _HILBERT_OPTIMIZATION_
#define _HILBERT_OPTIMIZATION_

#include "HilbertStrategy.h"
#include "GNode.h"
#include "PathOptimization.h"

class HilbertOptimization: public HilbertStrategy
{
    public:
        HilbertOptimization(CNode *root);
        ~HilbertOptimization();

        CNode* GetNextNode();
    private:

        GNode * graph;
        PathOptimization *optimizer;

};

#endif
