#include "ShortCutStrategy.h"
#include "GL/glut.h"
#include <stdio.h>

ShortCutStrategy::ShortCutStrategy(CNode *root)
{
    last = NULL;
    nodeStack.push_back(root);
}

ShortCutStrategy::~ShortCutStrategy()
{
    nodeStack.clear();
}

CNode* ShortCutStrategy::GetNextNode()
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
        // pop the visited node
        nodeStack.pop_back();

        if(last->children.empty()) //visited a leaf node
        {
            if(nodeStack.empty())
                return NULL;

            CNode * node = nodeStack.back();
            if(node->depth < last->depth )
            {
                if(node->waiting)
                {
                    if(node->VisitedInterestingDescendentExists())
                    {
                        CNode * ignoredParent = nodeStack.back();
                        nodeStack.pop_back();

                        for(unsigned int i=0; i<node->children.size(); i++)
                        {
                            if(!node->children[i]->waiting && !node->children[i]->visited)
                            {
                                nodeStack.push_back(node->children[i]);
                            }
                        }

                        nodeStack.push_back(ignoredParent); // it will be poped out in the next call to get next node
                        return GetNextNode();
                    }
                    else
                    {
                        //printf("*** NO interesting visited node exits.!!\n");
                        //nodeStack.pop_back();
                        last = node;
                        return node;
                    }
                }
                else
                {
                    std::vector<CNode*> branch;
                    node->GetNearestLeafAndParents(last->pos, branch);
                    if(branch.empty())
                    {
                        printf("Somthing weired happend !!!\n");
                        last = node;
                        return node;
                    }

                    node->waiting = true;

                    for(unsigned int i=0; i<branch.size(); i++)
                    {
                        CNode *pnode = branch[i];
                        pnode->waiting = true;
                        nodeStack.push_back(pnode);
                    }

                    last->waiting = false;
                    last = branch.back();
                    return branch.back();
                }
            }
            else
            {
                last = node;
                return node;
            }

        }
        else
        {
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

}

void ShortCutStrategy::glDraw()
{
//    if(nodeStack.size()<2)
//        return;

//    glColor3f(1,0,0);
//    glLineWidth(4);
//    glBegin(GL_LINES);
//    for(int i=0; i<nodeStack.size()-1;i++)
//    {
//        TooN::Vector<3> p1 = nodeStack[i+1]->GetMAVWaypoint();
//        TooN::Vector<3> p2 = nodeStack[i]->GetMAVWaypoint();

//        glVertex3f(p1[0],p1[1],p1[2]);
//        glVertex3f(p2[0],p2[1],p2[2]);
//    }
//    glEnd();

}
