#include "CNode.h"
#include <math.h>
#include <GL/glut.h>

float CNode::fov = 90 *3.14/(180);
int CNode::bf_sqrt = 2;
float CNode::minFootprintWidth = 1;


CNode::CNode(Rect target_foot_print):parent(NULL)
{
    isInteresting = true;
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
    children.push_back(cnode);
}

void CNode::PopulateChildren()
{
    double dl = (footPrint[2]-footPrint[0])/bf_sqrt;

    if(dl < minFootprintWidth)
        return;

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
        glLineWidth(2);
        glColor3f(.4,.8,.1);
        glBegin(GL_LINES);
        glVertex3f(parent->pos[0],parent->pos[1],parent->pos[2]);
        glVertex3f(pos[0],pos[1],pos[2]);
        glEnd();
    }

    glPointSize(5);
    glColor3f(0,0,0);
    glBegin(GL_POINTS);
    glVertex3f(pos[0],pos[1],pos[2]);
    glEnd();

    for(int i=0; i<children.size(); i++)
        children[i]->glDraw();
}

CNode* CNode::GetNearestLeaf(TooN::Vector<3> p)
{
    if(children.empty())
        return this;

    int minidx=0;
    double minDist = 99999999999;
    for(int i=0; i<children.size(); i++)
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
