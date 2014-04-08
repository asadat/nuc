#include "CNode.h"
#include <math.h>
#include <GL/glut.h>

#define IN(x,y)    (y[0] <= x[0] && x[0] <= y[2] && y[1] <= x[1] && x[1] <= y[3])


float CNode::fov = 90 *3.14/(180);
int CNode::bf_sqrt = 2;
float CNode::minFootprintWidth = 3;


CNode::CNode(Rect target_foot_print):parent(NULL)
{
    depth = 0;
    isInteresting = false;
    trueIsInteresting = false;
    footPrint = target_foot_print;
    pos[0] = (0.5)*(footPrint[0]+footPrint[2]);
    pos[1] = (0.5)*(footPrint[1]+footPrint[3]);
    pos[2] = (0.5)*fabs((footPrint[0]-footPrint[2])/tan(fov/2.0));

    PopulateChildren();
}

CNode::~CNode()
{

}

void CNode::CreateChildNode(Rect fp)
{
    CNode* cnode = new CNode(fp);
    cnode->parent = this;
    cnode->depth = depth +1;
    children.push_back(cnode);
}

void CNode::PopulateChildren()
{
    double fps = (footPrint[2]-footPrint[0]);
    if(fps <= minFootprintWidth)
        return;

    double dl = fps/bf_sqrt;

    for(int i=0; i<bf_sqrt; i++)
        for(int j=0; j<bf_sqrt; j++)
        {
            Rect fp;
            fp[0] = footPrint[0]+i*dl;
            fp[1] = footPrint[1]+j*dl;
            fp[2] = fp[0]+dl;
            fp[3] = fp[1]+dl;

            CreateChildNode(fp);

        }
}

void CNode::glDraw()
{
    if(parent != NULL)
    {
        glLineWidth(1);
        glColor3f(.2,.2,.2);
        glBegin(GL_LINES);
        glVertex3f(parent->pos[0],parent->pos[1],parent->pos[2]);
        glVertex3f(pos[0],pos[1],pos[2]);
        glEnd();
    }

    if(IsNodeInteresting())
    {
        glPointSize(6);
        glColor3f(0.6,0,0);
    }
    else
    {
        glPointSize(3);
        glColor3f(0,0,0);
    }

    glBegin(GL_POINTS);
    glVertex3f(pos[0],pos[1],pos[2]);
    glEnd();

    for(unsigned int i=0; i<children.size(); i++)
        children[i]->glDraw();

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    if(children.empty())
    {
        glColor3f(0.95,1,0.95);
        glBegin(GL_QUADS);
        glVertex3f(footPrint[0],footPrint[1], 0.1);
        glVertex3f(footPrint[0],footPrint[3], 0.1);
        glVertex3f(footPrint[2],footPrint[3], 0.1);
        glVertex3f(footPrint[2],footPrint[1], 0.1);
        glEnd();
    }

}

void CNode::PropagateDepth()
{
    if(parent == NULL)
        depth = 0;
    else
        depth = parent->depth +1;

    for(unsigned int i=0; i<children.size(); i++)
    {
        children[i]->PropagateDepth();
    }
}

bool CNode::PropagateInterestingness(Rect r)
{
    if(children.empty())
    {
        if(trueIsInteresting)
            return true;

        trueIsInteresting = IN(pos,r);

        return trueIsInteresting;
    }

    bool flag = false;
    for(unsigned int i=0; i<children.size(); i++)
    {
        bool res = children[i]->PropagateInterestingness(r);
        flag  = flag || res;
    }

    trueIsInteresting = trueIsInteresting || flag;

    return trueIsInteresting;
}

bool CNode::VisitedInterestingDescendentExists()
{
    if(children.empty())
        return (visited && IsNodeInteresting());

    bool flag = false;
    for(unsigned int i=0; i<children.size(); i++)
    {
        flag = children[i]->VisitedInterestingDescendentExists();
        if(flag)
            return true;
    }

    return false;
}

CNode* CNode::GetNearestLeaf(TooN::Vector<3> p)
{
    if(children.empty())
        return this;

    int minidx=0;
    double minDist = 99999999999;
    for(unsigned int i=0; i<children.size(); i++)
    {
        double dist = (children[i]->pos-p)*(children[i]->pos-p);
        if(dist < minDist)
        {
            minidx = i;
            minDist = dist;
        }
    }

    return children[minidx]->GetNearestLeaf(p);
}

void CNode::GetNearestLeafAndParents(TooN::Vector<3> p, std::vector<CNode*> & list)
{
    if(children.empty())
        return;

    int minidx=0;
    double minDist = 99999999999;
    for(unsigned int i=0; i<children.size(); i++)
    {
        double dist = (children[i]->pos-p)*(children[i]->pos-p);
        if(dist < minDist)
        {
            minidx = i;
            minDist = dist;
        }
    }

    list.push_back(children[minidx]);
    return children[minidx]->GetNearestLeafAndParents(p, list);
}
