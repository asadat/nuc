#include "SearchCoverageStrategy.h"
#include "GL/glut.h"
#include "NUCParam.h"
#include <math.h>
#include <vector>

#define ANGLE(a,b,c) (acos( ((a-b)*(c-b)) / (sqrt((a-b)*(a-b))*sqrt((c-b)*(c-b))) ))

using namespace std;
using namespace TooN;

SearchCoverageStrategy::SearchCoverageStrategy(CNode *root)
{
    cellW = 1;
    cluster_n=0;
    cutoff_prob = 0.60;

    tree = root;
    SetupGrid(root);    
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

void SearchCoverageStrategy::ReachedNode(CNode *node)
{
    double last_cutoff = 0;

    visitedNodes.push_back(node);
    if(fabs(last_cutoff-cutoff_prob) > 0.04)
    {
        last_cutoff = cutoff_prob;
        for(unsigned int i=0; i<visitedNodes.size(); i++)
            visitedNodes[i]->GenerateTargets(cutoff_prob);
    }
    else
    {
        node->GenerateTargets(cutoff_prob);
    }

    FindClusters();
}

void SearchCoverageStrategy::glDraw()
{
    for(int j=0; j<cluster_n; j++)
    {
        if(hulls[j]->size()<=1)
            continue;


        glColor3f(1,1,1);
        glLineWidth(4);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glBegin(GL_POLYGON);

        for(unsigned int i=0; i<hulls[j]->size();i++)
        {
            //glColor3f(RAND((i%5)/0.5,(i%5)/0.5+0.2),RAND(0,1),RAND(0,1));
            TooN::Vector<3> p1 = hulls[j]->at(i)->GetMAVWaypoint();
            glVertex3f(p1[0],p1[1],p1[2]);
        }
        glEnd();

//        glColor3f(1,0,0);
//        glLineWidth(8);
//        glBegin(GL_LINES);
//        TooN::Vector<3> p1 = hulls[j]->at(baseEdges[j].first)->GetMAVWaypoint();
//        TooN::Vector<3> p2 = hulls[j]->at(baseEdges[j].second)->GetMAVWaypoint();
//        glVertex3f(p1[0],p1[1],p1[2]);
//        glVertex3f(p2[0],p2[1],p2[2]);
//        glEnd();

//        if(lawnmovers.find((*hulls[j]->begin())->label) != lawnmovers.end())
//        {
//            vector<Vector<3> > & lm = *lawnmovers[(*hulls[j]->begin())->label];
//            glColor3f(0,0,1);
//            glLineWidth(3);
//            glBegin(GL_LINES);
//            for(int i=0; i<lm.size()-1; i++)
//            {
//               TooN::Vector<3> p1 = lm[i];
//               TooN::Vector<3> p2 = lm[i+1];
//               glVertex3f(p1[0],p1[1],p1[2]);
//               glVertex3f(p2[0],p2[1],p2[2]);
//            }
//            glEnd();
//        }

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

void SearchCoverageStrategy::SetupGrid(CNode *root)
{
    Vector<4> fp = root->GetNearestLeaf(makeVector(0,0,0))->footPrint;
    double dx = fp[2]-fp[0];
    double dy = fp[3]-fp[1];
    s = (root->footPrint[2]-root->footPrint[0])/dx;

    cellW = dx;


    double x0 = root->footPrint[0]+0.5*dx;
    double y0 = root->footPrint[1]+0.5*dy;

    for(double i=0; i<s; i+=1.0)
        for(double j=0; j<s; j+=1.0)
        {
            CNode * n = root->GetNearestLeaf(makeVector(x0+i*dx, y0+j*dy,0));
            n->grd_x = i;
            n->grd_y = j;
            grid.push_back(n);
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
        cutoff_prob = (cutoff_prob<1.0)?cutoff_prob+0.05:cutoff_prob;
        updateKey = true;
    }
    else if(key[';'])
    {
        cutoff_prob = (cutoff_prob>0.0)?cutoff_prob-0.05:cutoff_prob;
        updateKey = true;

    }
    else if(key['l'])
    {
        updateKey = true;

    }
}

void SearchCoverageStrategy::FindClusters()
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
            if(!(!GetNode(i,j)->extra_info && GetNode(i,j)->p_X > 0.99 && GetNode(i,j)->visited))
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
                    if( ne && !ne->extra_info && ne->p_X > 0.99 && GetNode(i,j)->visited)
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
    PlanLawnmovers();
}

void SearchCoverageStrategy::FindConvexHulls()
{
    baseEdges.clear();

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
        baseEdges.push_back(BaseEdge(*h));
    }
}

void SearchCoverageStrategy::ConvexHull(int label, vector<CNode*> & ch)
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
            break;
        else
            ch.push_back(v[next]);
    }


    // remove extra nodes (colinear edges)
    int sz = ch.size();

    for(int i=0; i < ch.size(); i++)
    {
        sz = ch.size();

        int i1 = ((i-1)+sz)%sz;
        int i2 = i;
        int i3 = (i+1)%sz;

        Vector<3> v = ch[i2]->pos - ch[i1]->pos;
        Vector<3> u = ch[i3]->pos - ch[i2]->pos;

        Vector<3> r = u^v;

        if(sqrt(r*r) < 0.1)
        {
            ch.erase(ch.begin()+i);
            i--;
        }
    }
}

double SearchCoverageStrategy::pointToLineDist(Vector<3> p1, Vector<3> p2, Vector<3> x)
{
    double d = fabs((p2[0]-p1[0])*(p1[1]-x[1]) - (p1[0]-x[0])*(p2[1]-p1[1]))/sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1]));
    return d;
}

std::pair<int, int> SearchCoverageStrategy::BaseEdge(std::vector<CNode*> & ch)
{
    int sz = ch.size();
    double minHeight = 999999;
    int min_idx = -1;

    for(int i=0; i < sz; i++)
    {
        double maxHeight = 0;

        for(int j=0; j < sz; j++)
        {
            if(j==i || (i+1)%sz ==j)
                continue;

            double h = pointToLineDist(ch[i]->GetMAVWaypoint(), ch[(i+1)%sz]->GetMAVWaypoint(), ch[j]->GetMAVWaypoint());
            maxHeight = (maxHeight < h) ? h : maxHeight;
        }

        if(minHeight > maxHeight)
        {
            minHeight = maxHeight;
            min_idx = i;
        }
    }

    return std::pair<int,int>(min_idx, (min_idx+1)%sz);
}

void SearchCoverageStrategy::PlanLawnmovers()
{
    // free vector objects
    while(!lawnmovers.empty())
    {
        pair<int, vector<Vector<3> >* > pr = *lawnmovers.begin();
        vector<Vector<3> > * h = pr.second;
        lawnmovers.erase(pr.first);
        h->clear();
        delete h;
    }

    for(int i=0; i < hulls.size(); i++)
    {
        vector<Vector<3> > * lm = new vector<Vector<3> >();
        PlanLawnmower(hulls[i], baseEdges[i].first, baseEdges[i].second, lm);
        lawnmovers[(*hulls[i]->begin())->label] = lm;
    }

    ROS_INFO("Planning Done.");
}

void SearchCoverageStrategy::PlanLawnmower(std::vector<CNode*> * ch, int baseStart_idx, int baseEnd_idx, std::vector<Vector<3> > * lm)
{
    int label = (*ch)[0]->label;
    Vector<3> baseDir  = ch->at(baseEnd_idx)->GetMAVWaypoint() - ch->at(baseStart_idx)->GetMAVWaypoint();
    Vector<3> baseDirNorm = baseDir;
    normalize(baseDirNorm);

    //clockwise orthogonal vector
    Vector<3> sweepDir = makeVector(baseDir[1], -baseDir[0], baseDir[2]);
    normalize(sweepDir);

    double step = 0.1;
    int n=-1;
    while(true)
    {
        if(n++ > 100)
            break;

        ROS_INFO("lawnmower track ... %d", lm->size());

        Vector<3> sn = ch->at(baseStart_idx)->GetMAVWaypoint();
        Vector<3> en = ch->at(baseEnd_idx)->GetMAVWaypoint();
        sn += n * cellW * sweepDir;
        en += n * cellW * sweepDir;

        CNode * tmp = NULL;
        CNode * tmp_old = NULL;

        bool atLeastOneTarget = false;

        tmp = tree->GetNearestLeaf(en);
        if(tmp->label == label)
        {
            atLeastOneTarget = true;
            while(tmp != tmp_old && tmp && tmp->label==label)
            {
                en += step*baseDirNorm;
                tmp_old = tmp;
                tmp = tree->GetNearestLeaf(en);
            }
        }
        else
        {
            while(tmp != tmp_old && tmp)
            {
                if(tmp->label!=label)
                {
                    en -= step*baseDirNorm;
                    tmp_old = tmp;
                    tmp = tree->GetNearestLeaf(en);
                }
                else
                {
                    atLeastOneTarget = true;
                }
            }
        }

        tmp = tree->GetNearestLeaf(sn);
        if(tmp->label == label)
        {
            atLeastOneTarget = true;

            while(tmp != tmp_old && tmp && tmp->label==label)
            {
                sn -= step*baseDirNorm;
                tmp_old = tmp;
                tmp = tree->GetNearestLeaf(sn);
            }
        }
        else
        {
            while(tmp != tmp_old && tmp)
            {
                if(tmp->label!=label)
                {
                    sn += step*baseDirNorm;
                    tmp_old = tmp;
                    tmp = tree->GetNearestLeaf(sn);
                }
                else
                {
                    atLeastOneTarget = true;
                }
            }
        }

        if(lm->size()%4 == 0)
        {
            lm->push_back(sn);
            lm->push_back(en);
        }
        else
        {
            lm->push_back(en);
            lm->push_back(sn);
        }

//        if((sn-en)*(sn-en) < 5*cellW)
//            break;
        if(!atLeastOneTarget)
            break;
    }

 }
