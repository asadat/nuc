#include "TestStrategy.h"
#include "GL/glut.h"
#include <stdio.h>
#include "NUCParam.h"

using namespace TooN;

TestStrategy::TestStrategy(CNode *root)
{
    Rect r = root->GetFootPrint();
    CNode *lb = new CNode(r);
    CNode *lt = new CNode(r);
    CNode *rb = new CNode(r);
    CNode *rt = new CNode(r);

    lb->pos = makeVector(r[0], r[1], root->pos[2]);
    lt->pos = makeVector(r[0], r[3], root->pos[2]);
    rt->pos = makeVector(r[2], r[3], root->pos[2]);
    rb->pos = makeVector(r[2], r[1], root->pos[2]);

    nodeStack.push_back(lb);
    nodeStack.push_back(lt);
    nodeStack.push_back(rt);
    nodeStack.push_back(rb);
    nodeStack.push_back(lb);

//    double l = (r[0]-r[2])*(r[0]-r[2]);
//    l = sqrt(l);
//    double ld = l;

//    int depth =0;
//    while(ld > NUCParam::min_footprint)
//    {
//        depth++;
//        ld /= CNode::bf_sqrt;
//    }

//    int n = l/ld;
//    Vector<3> startPos = root->GetNearestLeaf(makeVector(r[0],r[1],0))->GetPos();

//    printf("LM: n:%d l:%f ld:%f \n", n, l, ld);


//    for(int i=0; i< n; i++)
//        for(int j=0; j< n; j++)
//        {
//            int jj = (i%2 == 0)?j:n-j-1;
//            Vector<3> npos = startPos + makeVector(((double)i)*ld, ((double)jj)*ld, 0);
//            nodeStack.push_back(root->GetNearestLeaf(npos));
//        }
}

TestStrategy::~TestStrategy()
{

}

CNode* TestStrategy::GetNextNode()
{
    if(nodeStack.empty())
        return NULL;

    CNode* node = nodeStack.front();
    nodeStack.erase(nodeStack.begin());
    return node;
}

void TestStrategy::glDraw()
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
