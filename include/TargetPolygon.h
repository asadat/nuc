#ifndef _TARGETPOLYGON_H
#define _TARGETPOLYGON_H
#include "CNode.h"
#include <set>

using namespace std;
using namespace TooN;

class TargetPolygon
{
    public:
        TargetPolygon(vector<CNode*> &cs, CNode* parentNode);
        ~TargetPolygon();

        void glDraw();
        void GetLawnmowerPlan(vector<Vector<3> > &v);
        void MarkAsVisited();
        Vector<3> GetMiddlePos();
        void ReverseLawnmower();
        Vector<3> FirstLMPos(){return lm.front();}
        Vector<3> LastLMPos(){return lm.back();}
        size_t LawnmowerSize(){return lm.size();}
        void AddPolygon(TargetPolygon* p);
        int GetLabel(){return label;}
        bool IsNeighbour(TargetPolygon *tp);
        void SetVisited(bool visited_){visited = visited_;}

    private:
        void ProcessPolygon();
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
        set<CNode*> parentSearchNodes;
        double height;
        double cellW;
        bool visited;
        int base_idx[2];

};

#endif
