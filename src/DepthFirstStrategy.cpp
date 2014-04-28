#include "DepthFirstStrategy.h"
#include "GL/glut.h"

DepthFirstStrategy::DepthFirstStrategy(CNode *root)
{
    last = NULL;
    nodeStack.push_back(root);
}

DepthFirstStrategy::~DepthFirstStrategy()
{
    nodeStack.clear();
}

CNode* DepthFirstStrategy::GetNextNode()
{
    if(nodeStack.empty())
        return NULL;

    if(last == NULL)
    {
        last = nodeStack.back();
        return nodeStack.back();
    }
    else
    {
        nodeStack.pop_back();

        for(unsigned int i=0; i<last->children.size(); i++)
        {
            CNode* child = last->children[i];
            if(child->IsNodeInteresting())
            {
                nodeStack.push_back(child);
            }
        }

        if(nodeStack.empty())
            return NULL;

        last = nodeStack.back();
        return nodeStack.back();
    }
}

void DepthFirstStrategy::glDraw()
{
    if(nodeStack.size()<2)
        return;

    glColor3f(1,0,0);
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
