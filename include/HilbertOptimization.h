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
        void glDraw();
        void hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey);

    private:

        GNode * startNode;
        GNode * endNode;
        PathOptimization *optimizer;

        GNode::Path p;

        int draw_i_path;
};

#endif
