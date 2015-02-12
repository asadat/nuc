#ifndef _HILBERT_OPTIMIZATION_
#define _HILBERT_OPTIMIZATION_
#include "HilbertStrategy.h"

class HilbertOptimization: public HilbertStrategy
{
    public:
        HilbertOptimization(CNode *root);
        ~HilbertOptimization();

        CNode* GetNextNode();
    private:


};

#endif
