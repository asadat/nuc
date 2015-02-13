#ifndef _HILBERT_OPTIMIZATION_
#define _HILBERT_OPTIMIZATION_

#include "HilbertStrategy.h"
#include "GNode.h"
#include "PathOptimization.h"

class HilbertOptimization: public HilbertStrategy
{
    public:
        HilbertOptimization(CNode *root, TooN::Vector<3> init_pos, TooN::Vector<3> end_pos);
        ~HilbertOptimization();

        CNode* GetNextNode();
    private:

        GNode * startNode;
        GNode * endNode;
        PathOptimization *optimizer;

        Path p;
};

#endif
