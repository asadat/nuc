#ifndef _HILBERT_STARTEGY_
#define _HILBERT_STRATEGY_

#include "TraversalStrategy.h"
#include "LawnmowerStrategy.h"

#include "CNode.h"
#define MAX_HILBERT_ORDER 20

class HilbertStrategy: public TraversalStrategy
{
public:
    HilbertStrategy(CNode* root);
    ~HilbertStrategy();

    void glDraw();
    CNode* GetNextNode();
    void hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey);

private:
    bool IsLawnmowerBetter();
    bool UpdateLawnmowerDepth();
    bool UpdateIterator();
    void RotatePointOrderBy90(std::vector<TooN::Vector<3> >& list, bool cockwise);
    CNode * findNode(int x, int y, std::vector<CNode*> & list);
    int HilbertCurveOther(CNode* parent);
    void GenerateLawnmower(CNode* parentNode, std::vector<CNode*>& lm);
    CNode * GetFirstLMNode(CNode* node);
    CNode * GetLastLMNode(CNode* node);
    CNode * GetLMLParent(CNode* node);

    std::vector<CNode*> nodeStack;
    std::vector<CNode*> hilbert[MAX_HILBERT_ORDER];
    int waypointCount[MAX_HILBERT_ORDER];
    int LML;

    std::vector<CNode*> lmWps;
    //std::vector<TooN::Vector<3> > points;
    LawnmowerStrategy * lms;
    std::vector<CNode*>::iterator it;
    int lastDepth;
    int curDepth;
    int curCurve;
    bool isover;
};

#endif
