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

    void glDraw();

    static float fov;
    static int bf_sqrt;
    static float minFootprintWidth;
private:

    CNode *parent;
    std::vector<CNode*> children;

    Rect footPrint;
    TooN::Vector<3> pos;

};

#endif
