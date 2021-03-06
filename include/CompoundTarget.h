#ifndef _COMPOUND_TARGET_
#define _COMPOUND_TARGET_
#include <vector>
#include <TargetPolygon.h>

using namespace std;

class CompoundTarget
{
    public:
        CompoundTarget();
       ~CompoundTarget();

        void AddTarget(TargetPolygon* t);
        TargetPolygon * GetTarget(size_t i){return targets[i];}

        void glDraw();

        size_t size() const {return targets.size();}
        bool IsNonBoundary() const {return nonBoundary;}
        void ResetBoundaries();
        bool IsExtensible() const;
        void SetIsCurChildFlag(CNode *cur_search_node);
        void GetBoundarySeachNodes(vector<CNode*> &bnodes) const;
        void CalculateValue();
        void SetIgnored();
        void SetVisited();
        void GetLawnmowerPlan(vector<Vector<3> > &lm_plan) const;
        bool IsInside(const CNode *cell) const;
        double GetTrueValue() const;
    private:
        vector<TargetPolygon*> targets;

        bool nonBoundary;

        vector<CNode*> boundaryNodes[TargetPolygon::ALL];

        CNode* startNode;
        double value;
        double cost;
        bool cur_child; // shows if the compound target is partly a child of a search node

        friend class SearchCoverageStrategy;
};

#endif
