#ifndef _COMPOUND_TARGET_
#define _COMPOUND_TARGET_
#include <vector>
#include <TargetPolygon.h>

using namespace std;

class CompoundTarget
{
    public:
        CompoundTarget(){}
       ~CompoundTarget(){}

        void AddTarget(TargetPolygon* t);
        TargetPolygon * GetTarget(size_t i){return targets[i];}
        size_t size() const {return targets.size();}

    private:
        vector<TargetPolygon*> targets;

        friend class SearchCoverageStrategy;
};

#endif
