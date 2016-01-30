#ifndef _TARGETPOLYGON_H
#define _TARGETPOLYGON_H
#include "CNode.h"
#include <set>
#include <functional>

using namespace std;
using namespace TooN;

class TargetPolygon
{
    public:
        enum SIDE {L=0,R,U,D, ALL};
        TargetPolygon();
        TargetPolygon(vector<CNode*> &cs, CNode* parentNode, std::function<CNode*(int,int)> gn);
        ~TargetPolygon();

        void glDraw();
        void GetLawnmowerPlan(vector<Vector<3> > &v) const;
        void MarkAsVisited();
        void MarkIgnored();
        inline bool IsIgnored() const {return ignored;}
        inline bool IsVisited() const {return visited;}
        Vector<3> GetMiddlePos();
        void ReverseLawnmower();
        inline Vector<3> FirstLMPos() const {return lm.front();}
        inline Vector<3> LastLMPos() const {return lm.back();}
        inline size_t LawnmowerSize() const {return lm.size();}
        void AddPolygon(TargetPolygon* p, bool changeLabels=true, bool process=true);
        //int GetLabel(){return label;}
        bool IsNeighbour(TargetPolygon *tp);
        inline void SetVisited(bool visited_){visited = visited_;}
        double GetTargetRegionsArea() const;
        double GetTrueTargetRegionsArea() const;
        void GetCells(vector<CNode*> &v, const CNode *ofParent) const;
        inline void SetPolygonColor(Vector<3> color){pc=color;}

        void UpdateIsVisited();

        void OrBoundaryFlag(SIDE side, bool val);
        void SetBoundaryFlag(SIDE side, bool val);
        bool GetBoundaryFlag(SIDE side) const;
        bool IsNonBoundaryTarget() const;
        void ProcessPolygon();
        inline bool HasParent(CNode* par) const {return parentSearchNodes.find(par)!=parentSearchNodes.end();}
        inline Vector<3> GetCenter() const {return center;}
        bool IsInside(const CNode *cell) const;
        void FindApproximatePolygon();

    private:
        void ConvexHull();
        void FindBaseEdge();
        double pointToLineDist(TooN::Vector<3> p1, TooN::Vector<3> p2, TooN::Vector<3> x) const;
        double pointToLineSignedDist(TooN::Vector<3> p1, TooN::Vector<3> p2, TooN::Vector<3> x) const;
        bool GetLineSegmentIntersection(TooN::Vector<3> p1, TooN::Vector<3> p2, TooN::Vector<3> p3,
                                        TooN::Vector<3> p4, TooN::Vector<3> &intersection_p) const;
        void PlanLawnmower();
        void SetLawnmowerHeight();
        inline void PushLMwWypoint(TooN::Vector<3> v, const double wheight){lm.push_back(TooN::makeVector(v[0],v[1],wheight));}
        bool boundaryFLags[5];
        CNode* GetNeighbour_8(CNode* node, int i);

        bool ignored;
        int label;
        vector<CNode*> ch;
        vector<CNode*> cells;
        vector<Vector<3> > lm;
        set<CNode*> parentSearchNodes;
        double height;
        static double cellW;
        bool visited;
        int base_idx[2];
        Vector<3> pc;
        Vector<3> center;
        bool isLine;
        vector<CNode*> approxPoly;
        vector<CNode*> boundaryNodes;


        std::function<CNode*(int,int)> GetNode;

        friend class SearchCoverageStrategy;
};

#endif
