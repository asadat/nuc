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

        void glDraw();
        void GetLawnmowerPlan(vector<Vector<3> > &v);
        void MarkAsVisited();

    private:
        void ConvexHull();
        void FindBaseEdge();
        double pointToLineDist(TooN::Vector<3> p1, TooN::Vector<3> p2, TooN::Vector<3> x);
        double pointToLineSignedDist(TooN::Vector<3> p1, TooN::Vector<3> p2, TooN::Vector<3> x);
        bool GetLineSegmentIntersection(TooN::Vector<3> p1, TooN::Vector<3> p2, TooN::Vector<3> p3, TooN::Vector<3> p4, TooN::Vector<3> &intersection_p);
        void PlanLawnmower();

        int label;
        vector<CNode*> ch;
        vector<CNode*> cells;
        vector<Vector<3> > lm;
        double height;
        double cellW;

        int base_idx[2];
};

#endif
