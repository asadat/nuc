#include "BudgetedStrategy.h"
#include "GL/glut.h"
#include "NUCParam.h"
#include "Trajectory.h"
#include <math.h>

#define ANGLE(a,b,c) (acos( ((a-b)*(c-b)) / (sqrt((a-b)*(a-b))*sqrt((c-b)*(c-b))) ))

using namespace TooN;

BudgetedStrategy::BudgetedStrategy(CNode *root)
{
    cluster_n=0;
    drawDubins = false;
    cutoff_prob = 0.60;
    tree = root;

    SetupGrid(root);
    UpdateGridCutoff(tree);
    FindClusters();
    nodeStack.push_back(GetNode(0,0));

    double rad = (GetNode(0,0)->footPrint[2]-GetNode(0,0)->footPrint[0]);
    Trajectory::GenerateDubinTrajectory(makeVector(-10,-10), makeVector(10,-10), makeVector(10,-5), makeVector(-10,-5),rad, 1, dp);
    Trajectory::GenerateDubinTrajectory(makeVector(10,-5), makeVector(-10,-5), makeVector(-10,6), makeVector(10,0), rad, 1, dp);
    Trajectory::GenerateDubinTrajectory(makeVector(-10,6), makeVector(10,0), makeVector(10,8), makeVector(-10,8), 6*rad, 1, dp);
    Trajectory::GenerateDubinTrajectory(makeVector(10,8), makeVector(-10,8), makeVector(-10,10), makeVector(10,10), rad, 1, dp);
}

BudgetedStrategy::~BudgetedStrategy()
{

}

CNode* BudgetedStrategy::GetNextNode()
{
    if(nodeStack.empty())
        return NULL;

    CNode* node = nodeStack.front();
    nodeStack.erase(nodeStack.begin());
    return node;
}

void BudgetedStrategy::glDraw()
{

    if(drawDubins)
    {
        double h = (GetNode(0,0)->footPrint[2]-GetNode(0,0)->footPrint[0])/2;

        glColor3f(0.0,1,0.0);
        glLineWidth(4);
        glBegin(GL_LINES);
        for(unsigned int i=1; i<dp.size();i++)
        {
            TooN::Vector<2> p1 = dp[i];
            TooN::Vector<2> p2 = dp[i-1];
            if((p1-p2)*(p1-p2) > 2)
                glColor3f(0.0,1,0.0);
            else
                glColor3f(1,0,0.0);

            glVertex3f(p1[0],p1[1],2*h);
            glVertex3f(p2[0],p2[1],2*h);
        }
        glEnd();
    }

    for(int j=0; j<cluster_n; j++)
    {
        if(hulls[j]->size()<=1)
            continue;


        glColor3f(1,1,1);
        glLineWidth(4);
        glBegin(GL_POLYGON);

        for(unsigned int i=0; i<hulls[j]->size();i++)
        {
            TooN::Vector<3> p1 = hulls[j]->at(i)->GetMAVWaypoint();
            //TooN::Vector<3> p2 = hulls[j]->at((i==0)?hulls[j]->size()-1:i-1)->GetMAVWaypoint();

            //TooN::Vector<3> cl =hulls[j]->at(i)->colorBasis;

            //glColor3f(cl[0],cl[1],cl[2]);

            glVertex3f(p1[0],p1[1],2*p1[2]);
            //glVertex3f(p2[0],p2[1],2*p2[2]);
        }
        glEnd();
    }


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

void BudgetedStrategy::SetupGrid(CNode *root)
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

CNode * BudgetedStrategy::GetNode(int i, int j)
{
    int n = i*s+j;
    if(n < grid.size())
        return grid[n];
    else
        return NULL;
}

void BudgetedStrategy::FindClusters()
{
    clusters.clear();

    int cn = -1;
    for(int i=0; i<s; i++)
       for(int j=0; j<s; j++)
       {
          GetNode(i,j)->extra_info = false;
       }

    for(int i=0; i<s; i++)
    {
        for(int j=0; j<s; j++)
        {
            if(!(!GetNode(i,j)->extra_info && GetNode(i,j)->p_X > 0.99))
                continue;

            std::vector<CNode*> stack;
            stack.push_back(GetNode(i,j));
            cn++;
            if(c.size() < cn+1)
            {
                c.push_back(makeVector(RAND(0,1), RAND(0,1), RAND(0,1)));
            }

            while(!stack.empty())
            {
                CNode* nd = stack.back();
                stack.pop_back();
                nd->extra_info = true;
                nd->colorBasis = c[cn];
                nd->label = cn;
                clusters.insert(std::pair<int, CNode*>(cn, nd));

                for(int i=1; i<5; i++)
                {
                    CNode * ne = nd->GetNeighbourLeaf(i==1,i==2,i==3,i==4);
                    if( ne && !ne->extra_info && ne->p_X > 0.99)
                    {
                        ne->extra_info = true;
                        stack.push_back(ne);
                    }
                }
            }
        }
    }

    cluster_n = cn+1;

    FindConvexHulls();
}

void BudgetedStrategy::hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey)
{
    if(key['\''])
    {
        cutoff_prob = (cutoff_prob<1.0)?cutoff_prob+0.05:cutoff_prob;
        UpdateGridCutoff(tree);
        FindClusters();
        updateKey = false;
    }
    else if(key[';'])
    {
        cutoff_prob = (cutoff_prob>0.0)?cutoff_prob-0.05:cutoff_prob;
        UpdateGridCutoff(tree);
        FindClusters();
        updateKey = false;

    }
    else if(key['l'])
    {
        drawDubins = !drawDubins;
        updateKey = false;

    }
}


void BudgetedStrategy::UpdateGridCutoff(CNode* n)
{
    if(n->IsLeaf())
    {
        if(n->imgPrior >= cutoff_prob)
            n->SetPrior(1.0);
        else
            n->SetPrior(0.0);
    }
    else
    {
        for(unsigned int i=0; i < n->children.size(); i++)
            UpdateGridCutoff(n->children[i]);
    }
}

void BudgetedStrategy::FindConvexHulls()
{
    while(!hulls.empty())
    {
        vector<CNode*> * h = hulls.back();
        hulls.pop_back();
        h->clear();
        delete h;
    }

    for(int i=0; i<cluster_n; i++)
    {
        vector<CNode*> * h = new vector<CNode*>();
        ConvexHull(i, *h);
        hulls.push_back(h);
    }
}

void BudgetedStrategy::ConvexHull(int label, vector<CNode*> & ch)
{
    pair<multimap<int, CNode*>::iterator, multimap<int, CNode*>::iterator> seg_range;
    seg_range = clusters.equal_range(label);

    vector<CNode*> v;
    multimap<int, CNode*>::iterator it = seg_range.first;
    while(it != seg_range.second)
    {
        v.push_back(it->second);
        it++;
    }

    // first node is the bottom most node
    unsigned first=0;
    for(unsigned int i=1; i<v.size(); i++)
        if(v[first]->pos[1]> v[i]->pos[1])
            first = i;

    ch.push_back(v[first]);
    //v.erase(v.begin()+first);

    if(v.size()<=1)
        return;

    //second node
    unsigned second=1;
    Vector<3> p1 = ch[0]->pos - makeVector(-1, 0, 0);
    Vector<3> p2 = ch.back()->pos;

    double ang = ANGLE(p1, p2, v[1]->pos);
    for(unsigned int i=2; i<v.size(); i++)
    {
        double a = ANGLE(p1, p2, v[i]->pos);
        if(a> ang)
        {
            ang = a;
            second = i;
        }
    }

    ch.push_back(v[second]);

    if(v.size() <=2)
        return;

    //int n=0;
    // the rest of the nodes
    while(true)
    {
        //if(n++ > 100) return;

        p1 = ch[ch.size()-2]->pos;
        p2 = ch.back()->pos;
        int next = -1;
        ang = -1;
        for(unsigned int i=0; i<v.size(); i++)
        {
            if(v[i] == ch.back())
                continue;

            double a = ANGLE(p1, p2, v[i]->pos);
            if(a> ang)
            {
                ang = a;
                next = i;
            }
        }

        if(find(ch.begin(), ch.end(), v[next]) != ch.end())
            return;
        else
            ch.push_back(v[next]);

    }

}
