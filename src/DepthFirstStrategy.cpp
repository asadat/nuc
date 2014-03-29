#include "DepthFirstStrategy.h"
#include "GL/glut.h"

DepthFirstStrategy::DepthFirstStrategy(CNode *root)
{
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

    CNode * node = nodeStack.back();
    nodeStack.pop_back();

    for(int i=0; i<node->children.size(); i++)
    {
        CNode* child = node->children[i];
        if(child->IsNodeInteresting())
        {
            nodeStack.push_back(child);
        }
    }

    return node;
}

void DepthFirstStrategy::glDraw()
{
    if(nodeStack.size()<2)
        return;

    glColor3f(1,0,0);
    glLineWidth(4);
    glBegin(GL_LINES);
    for(int i=0; i<nodeStack.size()-1;i++)
    {
        TooN::Vector<3> p1 = nodeStack[i+1]->GetPos();
        TooN::Vector<3> p2 = nodeStack[i]->GetPos();

        glVertex3f(p1[0],p1[1],p1[2]);
        glVertex3f(p2[0],p2[1],p2[2]);
    }
    glEnd();

}
