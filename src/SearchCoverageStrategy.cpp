#include "SearchCoverageStrategy.h"
#include "GL/glut.h"
#include "NUCParam.h"
#include <math.h>

#define ANGLE(a,b,c) (acos( ((a-b)*(c-b)) / (sqrt((a-b)*(a-b))*sqrt((c-b)*(c-b))) ))

using namespace TooN;

SearchCoverageStrategy::SearchCoverageStrategy(CNode *root)
{
    ROS_INFO("SearchCoverageStrategy creating...");

    tree = root;

    SetupGrid(root);

    nodeStack.push_back(GetNode(0,0));

    ROS_INFO("SearchCoverageStrategy created.");


}

SearchCoverageStrategy::~SearchCoverageStrategy()
{

}

CNode* SearchCoverageStrategy::GetNextNode()
{

    if(nodeStack.empty())
        return NULL;

    CNode* node = nodeStack.front();
    nodeStack.erase(nodeStack.begin());
    return node;
}

void SearchCoverageStrategy::glDraw()
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

void SearchCoverageStrategy::SetupGrid(CNode *root)
{
    Vector<4> fp = root->GetNearestLeaf(makeVector(0,0,0))->footPrint;
    double dx = fp[2]-fp[0];
    double dy = fp[3]-fp[1];
    s = (root->footPrint[2]-root->footPrint[0])/dx;

    //ROS_INFO("GRID: %f %f", dx, dy);

    double x0 = root->footPrint[0]+0.5*dx;
    double y0 = root->footPrint[1]+0.5*dy;

    //ROS_INFO("s: %d", s);

    //ROS_INFO("X0 Y0: %f %f", x0, y0);

    for(double i=0; i<s; i+=1.0)
        for(double j=0; j<s; j+=1.0)
        {
            grid.push_back(root->GetNearestLeaf(makeVector(x0+i*dx, y0+j*dy,0)));
            //ROS_INFO("X Y: %f %f", i, j);
        }
}

CNode * SearchCoverageStrategy::GetNode(int i, int j)
{
    int n = i*s+j;
    if(n < grid.size())
        return grid[n];
    else
        return NULL;
}

void SearchCoverageStrategy::hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey)
{
    if(key['\''])
    {

        updateKey = false;
    }
    else if(key[';'])
    {

        updateKey = false;

    }
    else if(key['l'])
    {
        updateKey = false;

    }
}
