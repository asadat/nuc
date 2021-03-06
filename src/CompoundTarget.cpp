#include "CompoundTarget.h"
#include <boost/bind.hpp>
#include <GL/glut.h>
#include "ros/ros.h"

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

    glLineStipple(1, 0x00FF);
    glEnable(GL_LINE_STIPPLE);
    glColor4f(0,0,0,0.8);
    glLineWidth(3);
    glBegin(GL_LINES);
    for(int i=0; i<TargetPolygon::ALL; i++)
    {
        for(size_t j=0; j<boundaryNodes[i].size(); j++)
        {
            if(!targets.empty())
            {
                Vector<3> p1 = targets.front()->GetCenter();
                Vector<3> p2 = boundaryNodes[i][j]->GetMAVWaypoint();
                glVertex3f(p1[0], p1[1], p1[2]);
                glVertex3f(p2[0], p2[1], p2[2]);
            }
        }
    }
    glEnd();
    glDisable(GL_LINE_STIPPLE);
}

void CompoundTarget::ResetBoundaries()
{
    for(size_t j=0; j<targets.size(); j++)
    {
        targets[j]->SetBoundaryFlag(TargetPolygon::ALL, false);
    }

    for(size_t j=0; j<TargetPolygon::ALL; j++)
    {
        boundaryNodes[j].clear();
    }
}

void CompoundTarget::SetIsCurChildFlag(CNode* cur_search_node)
{
    cur_child = false;
    for(size_t i=0; i<targets.size() && !cur_child; i++)
    {
        if(targets[i]->HasParent(cur_search_node))
            cur_child = true;
    }
}

bool CompoundTarget::IsExtensible() const
{
    for(size_t j=0; j<TargetPolygon::ALL; j++)
    {
        if(!boundaryNodes[j].empty())
            return true;
    }

    return false;
}

void CompoundTarget::GetBoundarySeachNodes(vector<CNode *> &bnodes) const
{
    for(size_t j=0; j<TargetPolygon::ALL; j++)
    {
        copy(boundaryNodes[j].begin(), boundaryNodes[j].end(), back_inserter(bnodes));
    }
}


void CompoundTarget::CalculateValue()
{
    //ROS_INFO("compound target value:");
    value = 0;
    for(size_t i=0; i<targets.size(); i++)
    {
        value += targets[i]->GetTargetRegionsArea();
    }
}

void CompoundTarget::SetIgnored()
{
    for_each(targets.begin(), targets.end(), boost::bind(&TargetPolygon::MarkIgnored, _1));
}
void CompoundTarget::SetVisited()
{
    for_each(targets.begin(), targets.end(), boost::bind(&TargetPolygon::MarkAsVisited, _1));
}

void CompoundTarget::GetLawnmowerPlan(vector<Vector<3> > &lm_plan) const
{
    for(size_t i=0; i<targets.size(); i++)
    {
        targets[i]->GetLawnmowerPlan(lm_plan);
    }
}

double CompoundTarget::GetTrueValue() const
{
    double sum = 0;
    for(const auto &c:targets)
        sum += c->GetTrueTargetRegionsArea();

    return sum;
}

bool CompoundTarget::IsInside(const CNode *cell) const
{
    for(size_t i=0; i<targets.size(); i++)
    {
        if(targets[i]->IsInside(cell))
            return true;
    }

    return false;
}
