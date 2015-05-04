#include "SearchCoverageStrategy.h"
#include "GL/glut.h"
#include "NUCParam.h"
#include <math.h>
#include <vector>
#include "TargetPolygon.h"
#include "TSP.h"

#define D2(a,b) (a-b)*(a-b)

using namespace std;
using namespace TooN;

SearchCoverageStrategy::SearchCoverageStrategy(CNode *root)
{
    dummy = new CNode(makeVector(0,0,0,0));
    dummy->visited = false;
    dummy->depth = dummy->maxDepth;

    //cellW = 1;
    cluster_n=0;
    cutoff_prob = 0.60;

    tree = root;
    tree->SetTreeVisited(false);
    SetupGrid(root);    
    GenerateLawnmower();
}

SearchCoverageStrategy::~SearchCoverageStrategy()
{
    delete dummy;
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
    if(!target_lms.empty())
    {
        dummy->pos = target_lms.front();
        dummy->visited = false;
        target_lms.erase(target_lms.begin());
        return dummy;
    }

    if(nodeStack.empty())
        return NULL;

    CNode* node = nodeStack.front();
    nodeStack.erase(nodeStack.begin());
    return node;
}

void SearchCoverageStrategy::ReachedNode(CNode *node)
{
    bool reachedSearchNode = (node->depth == (node->maxDepth - NUCParam::lm_height));

    int last_c = 0;

    if(reachedSearchNode)
    {
        // in the NUC upon reaching a node all the descendants are visited
        // which is not what we want in this specific strategy, hence the
        // following hack
        for(size_t i=0; i<node->children.size(); i++)
            node->children[i]->SetTreeVisited(false);

        node->SetAncestorVisited(true);

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

        last_c = targets.size();
        FindClusters(true);

        //ROS_INFO("#clusters: %u", targets.size());
    }

    if(NUCParam::policy == "greedy")
    {
        if(reachedSearchNode)
        {
            for(size_t i=last_c; i < targets.size(); i++)
            {
                targets[i]->GetLawnmowerPlan(target_lms);
                targets[i]->MarkAsVisited();
            }
        }
    }
    else if(NUCParam::policy == "delayed_greedy")
    {

    }
    else if(NUCParam::policy == "delayed")
    {
        if(reachedSearchNode)
        {
            if(nodeStack.empty())
            {
                CleanupTargets();
                FindClusters(false);


                vector<Entity*> t_list;
                vector<Entity*> tsp_list;

                Entity * sn = new Entity();
                sn->start = true;
                sn->end = false;
                sn->pos = node->GetMAVWaypoint();
                t_list.push_back(sn);

                Entity * en = new Entity();
                en->start = false;
                en->end = true;
                en->pos = visitedNodes.front()->GetMAVWaypoint();
                t_list.push_back(en);

                for(size_t i=0; i < targets.size(); i++)
                {
                    if(targets[i]->LawnmowerSize() <= 0)
                        continue;

                    Entity * n = new Entity();
                    n->start = false;
                    n->end = false;
                    n->pos = targets[i]->GetMiddlePos();
                    n->nodeIdx = i;
                    t_list.push_back(n);
                }

                TSP tsp;
                tsp.GetShortestPath(t_list, tsp_list);

                vector<TargetPolygon*> tsp_target;
                for(size_t i=0; i < tsp_list.size(); i++)
                {
                    if(tsp_list[i]->start || tsp_list[i]->end)
                        continue;
                    tsp_target.push_back(targets[tsp_list[i]->nodeIdx]);
                }

                // optimizing target lawnmower start and end
                for(int i=0; i < tsp_target.size(); i++)
                {
                    bool flag = false;

                    if(i==0)
                    {
                        if(D2(node->GetMAVWaypoint(), tsp_target[i]->FirstLMPos()) + D2(tsp_target[i]->LastLMPos(), tsp_target[i+1]->FirstLMPos()) >
                                D2(node->GetMAVWaypoint(), tsp_target[i]->LastLMPos()) + D2(tsp_target[i]->FirstLMPos(), tsp_target[i+1]->FirstLMPos()))
                        {
                            tsp_target[i]->ReverseLawnmower();
                            flag = true;
                        }
                    }
                    else if(i == tsp_target.size()-1)
                    {
                        if(D2(tsp_target[i-1]->LastLMPos(), tsp_target[i]->FirstLMPos()) + D2(tsp_target[i]->LastLMPos(), en->pos) >
                                D2(tsp_target[i-1]->LastLMPos(), tsp_target[i]->LastLMPos()) + D2(tsp_target[i]->FirstLMPos(), en->pos))
                        {
                            tsp_target[i]->ReverseLawnmower();
                            flag = true;
                        }
                    }
                    else
                    {
                        if(D2(tsp_target[i-1]->LastLMPos(), tsp_target[i]->FirstLMPos()) + D2(tsp_target[i]->LastLMPos(), tsp_target[i+1]->FirstLMPos()) >
                                D2(tsp_target[i-1]->LastLMPos(), tsp_target[i]->LastLMPos()) + D2(tsp_target[i]->FirstLMPos(), tsp_target[i+1]->FirstLMPos()))
                        {
                            tsp_target[i]->ReverseLawnmower();
                            flag = true;
                        }
                    }

                    if(flag)
                        i++;
                }

                for(size_t i=0; i < tsp_target.size(); i++)
                {
                    tsp_target[i]->GetLawnmowerPlan(target_lms);
                    tsp_target[i]->MarkAsVisited();
                }


                delete sn;
                delete en;

                tsp_list.clear();
                while(t_list.empty())
                {
                    Entity* e = t_list.back();
                    t_list.pop_back();
                    delete e;
                }

                reverse(target_lms.begin(), target_lms.end());
            }
        }
    }

}

void SearchCoverageStrategy::glDraw()
{
    for(int j=0; j<cluster_n; j++)
        targets[j]->glDraw();


    glColor3f(0.3,0.8,0.9);
    glLineWidth(2);
    glBegin(GL_LINES);
    for(unsigned int i=1; i<target_lms.size();i++)
    {
        TooN::Vector<3> p1 = target_lms[i-1];
        TooN::Vector<3> p2 = target_lms[i];

        glVertex3f(p1[0],p1[1],p1[2]);
        glVertex3f(p2[0],p2[1],p2[2]);
    }
    glEnd();

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

    //cellW = dx;


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
    {
        ROS_WARN("GetNode: out of boundary access ...");
        return NULL;
    }
}

void SearchCoverageStrategy::hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey)
{
    bool flag = false;
    if(key['\''])
    {
        cutoff_prob = (cutoff_prob<1.0)?cutoff_prob+0.05:cutoff_prob;
        updateKey = true;
        flag = true;
    }
    else if(key[';'])
    {
        cutoff_prob = (cutoff_prob>0.0)?cutoff_prob-0.05:cutoff_prob;
        updateKey = true;
        flag = true;
    }
    else if(key['l'])
    {
        updateKey = true;
    }

    if(flag)
    {
        for(unsigned int i=0; i<visitedNodes.size(); i++)
            visitedNodes[i]->GenerateTargets(cutoff_prob);
        FindClusters(true);
    }


}

void SearchCoverageStrategy::CleanupTargets()
{
    while(!targets.empty())
    {
        TargetPolygon * p = targets.back();
        targets.pop_back();
        delete p;
    }

    clusters.clear();
    cluster_n = 0;
}

void SearchCoverageStrategy::FindClusters(bool incremental)
{
    //ROS_INFO("Find Cluster started.");

    //clusters.clear();
  //  CleanupTargets();

    int cn = targets.size()-1;
    for(int i=0; i<s; i++)
       for(int j=0; j<s; j++)
       {
           if(!incremental || GetNode(i,j)->label==-1)
           {
                GetNode(i,j)->extra_info = false;
                GetNode(i,j)->label = -1;
           }
       }

    for(int i=0; i<s; i++)
    {
        for(int j=0; j<s; j++)
        {
            if(GetNode(i,j)->extra_info || GetNode(i,j)->p_X < 0.99 || GetNode(i,j)->visited || !GetNode(i,j)->ancestor_visited)
                continue;

            std::vector<CNode*> stack;
            stack.push_back(GetNode(i,j));
            cn++;
           // ROS_INFO("Cluster added.");
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

                for(int ii=1; ii<5; ii++)
                {
                    CNode * ne = nd->GetNeighbourLeaf(ii==1,ii==2,ii==3,ii==4);
                    if( ne && !ne->extra_info && ne->p_X > 0.95 && !ne->visited && ne->ancestor_visited)
                    {
                        ne->extra_info = true;
                        stack.push_back(ne);
                    }
                }
            }
        }
    }

    cluster_n = cn+1;
    size_t old_cluster_n = targets.size();
    for(size_t i = old_cluster_n; i< cluster_n; i++)
    {
        pair<multimap<int, CNode*>::iterator, multimap<int, CNode*>::iterator> seg_range;
        seg_range = clusters.equal_range(i);

        vector<CNode*> v;
        multimap<int, CNode*>::iterator it = seg_range.first;
        while(it != seg_range.second)
        {
            v.push_back(it->second);
            it++;
        }

        TargetPolygon *t = new TargetPolygon(v, visitedNodes.back());
        targets.push_back(t);
    }
}
