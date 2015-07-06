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
        bool IsNonBoundary(){return nonBoundary;}
        void ResetBoundaries();
        bool IsExtensible();


    private:
        vector<TargetPolygon*> targets;
        bool nonBoundary;

        vector<CNode*> boundaryNodes[TargetPolygon::ALL];

        CNode* startNode;
        double value;
        double cost;

        friend class SearchCoverageStrategy;
};

#endif
