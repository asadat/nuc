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
    void hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey);

private:
    bool UpdateIterator();
    void RotatePointOrderBy90(std::vector<TooN::Vector<3> >& list, bool cockwise);
    CNode * findNode(int x, int y, std::vector<CNode*> & list);
    int HilbertCurveOther(CNode* parent);

    std::vector<CNode*> nodeStack;
    std::vector<CNode*> hilbert[100];
    //std::vector<TooN::Vector<3> > points;

    std::vector<CNode*>::iterator it;
    int lastDepth;
    int curDepth;
    int curCurve;
    bool isover;
};

#endif
