#ifndef _HILBERT_STARTEGY_
#define _HILBERT_STRATEGY_

#include "TraversalStrategy.h"
#include "CNode.h"

class HilbertStrategy: public TraversalStrategy
{
public:
    HilbertStrategy(CNode* root);
    ~HilbertStrategy();

    void glDraw();
    CNode* GetNextNode();

private:
    void RotateOrderBy90(std::vector<TooN::Vector<2, int> >& list, bool cockwise);
    CNode * findNode(int x, int y, std::vector<CNode*> & list);
    int HilbertCurve(CNode* parent);
    std::vector<CNode*> nodeStack;
    std::vector<CNode*> hilbert[100];
    std::vector<TooN::Vector<3> > points;
};

#endif
