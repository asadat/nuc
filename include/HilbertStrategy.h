#ifndef _HILBERT_STARTEGY_
#define _HILBERT_STRATEGY_

#include "TraversalStrategy.h"
#include "CNode.h"
#define MAX_HILBERT_ORDER 10

class HilbertStrategy: public TraversalStrategy
{
public:
    HilbertStrategy(CNode* root);
    ~HilbertStrategy();

    void glDraw();
    CNode* GetNextNode();
    void hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey);

protected:
    bool UpdateIterator();
    void RotatePointOrderBy90(std::vector<TooN::Vector<3> >& list, bool cockwise);
    CNode * findNode(int x, int y, std::vector<CNode*> & list);
    int HilbertCurveOther(CNode* parent);

    std::vector<CNode*> nodeStack;
    std::vector<CNode*> hilbert[MAX_HILBERT_ORDER];
    int waypointCount[MAX_HILBERT_ORDER];
    //std::vector<TooN::Vector<3> > points;

    std::vector<CNode*>::iterator it;
    int lastDepth;
    int curDepth;
    int curCurve;
    bool isover;
};

#endif
