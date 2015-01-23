#include "LawnmowerStrategy.h"
#include "GL/glut.h"
#include <stdio.h>
#include "NUCParam.h"

using namespace TooN;

LawnmowerStrategy::LawnmowerStrategy(CNode *root, CNode* enterN, CNode* exitN)
{
    if(enterN==NULL && exitN==NULL)
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
    else
    {
        CNode * firstN = enterN;
        CNode * lastN  = exitN;


        double l = fabs(firstN->footPrint[0]-firstN->footPrint[2]);

        Vector<2> interlapVec = makeVector(lastN->pos[0]-firstN->pos[0], lastN->pos[1]-firstN->pos[1]);
        normalize(interlapVec);
        Vector<2> testVec = makeVector(root->pos[0]-lastN->pos[0], root->pos[1]-lastN->pos[1]);
        normalize(testVec);
        Vector<2> interWp = makeVector(-interlapVec[1], interlapVec[0]);
        if( interWp*testVec < 0)
            interWp = makeVector(interlapVec[1], -interlapVec[0]);

        int n = floor((fabs(root->footPrint[0]-root->footPrint[2])+0.2)/l);

        Vector<3> interlapVec3 = l * makeVector(interlapVec[0], interlapVec[1], 0);
        Vector<3> interWp3 = l * makeVector(interWp[0], interWp[1], 0);

        ROS_INFO("LM: area:%.1f fp:%.1f", fabs(root->footPrint[0]-root->footPrint[2]), l);
        ROS_INFO("interlap: %f %f %f", interlapVec3[0], interlapVec3[1], interlapVec3[2]);
        ROS_INFO("interwp: %f %f %f", interWp3[0], interWp3[1], interWp3[2]);

        for(int i=0; i< n; i++)
            for(int j=0; j< n; j++)
            {
                int jj = (i%2 == 0)?j:n-j-1;
                Vector<3> npos = firstN->pos + i * interlapVec3 + jj * interWp3;
                nodeStack.push_back(root->GetNearestLeaf(npos));
            }
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
