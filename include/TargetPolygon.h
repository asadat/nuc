#ifndef _TARGETPOLYGON_H
#define _TARGETPOLYGON_H
#include "CNode.h"

using namespace std;
using namespace TooN;

class TargetPolygon
{
    public:
        TargetPolygon(vector<CNode*> &cs);
        ~TargetPolygon();

    private:
        void ConvexHull();

        int label;
        vector<CNode*> ch;
        vector<CNode*> cells;
        vector<Vector<3> > lm;
        double height;

        int base_idx[2];
};

#endif
