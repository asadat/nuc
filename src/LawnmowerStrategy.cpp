#include "LawnmowerStrategy.h"
#include "GL/glut.h"
#include <stdio.h>
#include "NUCParam.h"

using namespace TooN;

LawnmowerStrategy::LawnmowerStrategy(CNode *root)
{
    Rect r = root->GetFootPrint();
    double l = (r[0]-r[2])*(r[0]-r[2]);
    l = sqrt(l);
    double ld = l;

    int depth =0;
    while(ld > NUCParam::min_footprint)
    {
        depth++;
        ld /= NUCParam::bf_sqrt;
    }

    int n = l/ld;
    Vector<3> startPos = root->GetNearestLeaf(makeVector(r[0],r[1],0))->GetPos();

    printf("LM: n:%d l:%f ld:%f \n", n, l, ld);

    for(int i=0; i< n; i++)
        for(int j=0; j< n; j++)
        {
            int jj = (i%2 == 0)?j:n-j-1;
            Vector<3> npos = startPos + makeVector(((double)i)*ld, ((double)jj)*ld, 0);
            nodeStack.push_back(root->GetNearestLeaf(npos));
        }
}

LawnmowerStrategy::~LawnmowerStrategy()
{

}

CNode* LawnmowerStrategy::GetNextNode()
{
    if(nodeStack.empty())
        return NULL;

    CNode* node = nodeStack.front();
    nodeStack.erase(nodeStack.begin());
    return node;
}

void LawnmowerStrategy::glDraw()
{
    if(nodeStack.size() < 2)
        return;

    glColor3f(0.5,0.6,0.6);
    glLineWidth(4);
    glBegin(GL_LINES);
    for(unsigned int i=0; i<nodeStack.size()-1;i++)
    {
        TooN::Vector<3> p1 = nodeStack[i+1]->GetMAVWaypoint();
        TooN::Vector<3> p2 = nodeStack[i]->GetMAVWaypoint();

        glVertex3f(p1[0],p1[1],p1[2]);
        glVertex3f(p2[0],p2[1],p2[2]);
    }
    glEnd();

}
