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
    private:
        vector<TargetPolygon*> targets;
        bool nonBoundary;
        friend class SearchCoverageStrategy;
};

#endif
