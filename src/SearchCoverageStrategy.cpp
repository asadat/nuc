#include "SearchCoverageStrategy.h"
#include "GL/glut.h"
#include "NUCParam.h"
#include <math.h>

#define ANGLE(a,b,c) (acos( ((a-b)*(c-b)) / (sqrt((a-b)*(a-b))*sqrt((c-b)*(c-b))) ))

using namespace TooN;

SearchCoverageStrategy::SearchCoverageStrategy(CNode *root)
{

    tree = root;

    SetupGrid(root);

    nodeStack.push_back(GetNode(0,0));

    GenerateLawnmower();

}

SearchCoverageStrategy::~SearchCoverageStrategy()
{

}

void SearchCoverageStrategy::GenerateLawnmower()
{
    Rect r = tree->GetFootPrint();
    double l = (r[0]-r[2])*(r[0]-r[2]);
    l = sqrt(l);

    CNode * startNode = tree->GetNearestLeaf(makeVector(r[0],r[1],0), NUCParam::lm_height);
    ROS_INFO("lm_height: %d depth: %d maxdepth: %d", NUCParam::lm_height, startNode->depth, CNode::maxDepth);
    Vector<3> startPos = startNode->GetPos();

    Rect rc = startNode->GetFootPrint();
    double ld = sqrt((rc[0]-rc[2])*(rc[1]-rc[3]));
    int n = l/ld;


    printf("LM: n:%d l:%f ld:%f \n", n, l, ld);

    for(int i=0; i< n; i++)
        for(int j=0; j< n; j++)
        {
            int jj = (i%2 == 0)?j:n-j-1;
            Vector<3> npos = startPos + makeVector(((double)i)*ld, ((double)jj)*ld, 0);
            nodeStack.push_back(tree->GetNearestLeaf(npos,  NUCParam::lm_height));
        }
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
