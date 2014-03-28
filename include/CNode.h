#ifndef _CNODE_
#define _CNODE_
#include <vector>
#include "TooN/TooN.h"

typedef TooN::Vector<4> Rect;

class CNode
{
public:
    CNode(Rect target_foot_print);
    ~CNode();

    // coverage related methods
    void PopulateChildren();

    // tree related methods
    void CreateChildNode(Rect fp);

    bool IsNodeInteresting(){return isInteresting;}
    void SetIsInteresting(bool interesting){isInteresting=interesting;}
    TooN::Vector<3> GetPos(){return pos;}
    void glDraw();

    CNode* GetNearestLeaf(TooN::Vector<3> p);
    Rect GetFootPrint(){return footPrint;}

    static float fov;
    static int bf_sqrt;
    static float minFootprintWidth;
    std::vector<CNode*> children;


private:

    CNode *parent;


    Rect footPrint;
    TooN::Vector<3> pos;
    bool isInteresting;
};

#endif
