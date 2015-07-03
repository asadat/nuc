#include "CompoundTarget.h"

CompoundTarget::CompoundTarget()
{
    nonBoundary = true;
}

CompoundTarget::~CompoundTarget()
{

}

void CompoundTarget::AddTarget(TargetPolygon *t)
{
    targets.push_back(t);
    nonBoundary = nonBoundary && t->IsNonBoundaryTarget();
}

void CompoundTarget::glDraw()
{
    for(size_t j=0; j<targets.size(); j++)
    {
        if(nonBoundary)
            targets[j]->SetPolygonColor(makeVector(0,1,0));

        targets[j]->glDraw();
    }


}
