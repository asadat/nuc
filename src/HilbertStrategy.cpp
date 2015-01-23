#include "HilbertStrategy.h"
#include "GL/glut.h"
#include <stdio.h>
#include "NUCParam.h"

using namespace TooN;


HilbertStrategy::HilbertStrategy(CNode *root)
{
    for(unsigned int i=0; i< MAX_HILBERT_ORDER; i++)
        waypointCount[i] = 0;

    assert(NUCParam::bf_sqrt == 2);

    isover = false;
    lastDepth = HilbertCurveOther(root);
    LML = lastDepth-2;

    lms = NULL;

}

int HilbertStrategy::HilbertCurveOther(CNode *parent)
{
    Rect r = parent->GetFootPrint();
    double l = fabs(r[0]-r[2]);

    int depth =1;
    while(l > NUCParam::min_footprint)
    {
        depth++;
        l /= NUCParam::bf_sqrt;
    }

    if(depth>99)
        depth = 99;

    ROS_INFO("depth: %d", depth);

    hilbert[0].push_back(parent);

    hilbert[1].push_back(findNode(0,1,parent->children));
    hilbert[1].push_back(findNode(0,0,parent->children));
    hilbert[1].push_back(findNode(1,0,parent->children));
    hilbert[1].push_back(findNode(1,1,parent->children));

    for(int lvl=2; lvl < depth; lvl++)
    {
        for(int k=0; k<4; k++)
        {
           std::vector<Vector<3> > order;
           for(unsigned int i=0; i<hilbert[lvl-1].size(); i++)
           {
               if(k==1 || k== 2)
                   order.push_back(0.5 * hilbert[lvl-1][i]->pos);
               else
                   order.insert(order.begin(),(0.5 * hilbert[lvl-1][i]->pos));
           }

           if(k == 0)
               RotatePointOrderBy90(order, false);

           if(k == 3)
               RotatePointOrderBy90(order, true);

           double s = sqrt(order.size());
           double dx = (fabs(parent->footPrint[0]-parent->footPrint[2])*0.5)/s;
           //double dy = (fabs(parent->footPrint[1]-parent->footPrint[3])*0.5)/s;

           double DX,DY;
           if(k == 0)
           {    DX=-0.5; DY=0.5;}
           else if(k == 1)
           {    DX=-0.5; DY=-0.5;}
           else if(k == 2)
           {    DX=0.5; DY=-0.5;}
           else if(k == 3)
           {    DX=0.5; DY=0.5;}

           for(unsigned int i=0; i<order.size();i++)
           {
               Vector<3> p = order[i];

               p[0] += DX*fabs(parent->footPrint[0]-parent->footPrint[2])*0.5;
               p[1] += DY*fabs(parent->footPrint[1]-parent->footPrint[3])*0.5;

               p[2] = (0.5)*fabs((dx)/tan(NUCParam::FOV/2.0));

               hilbert[lvl].push_back(parent->GetNearestNode(p));

//               if(lvl+1 == depth)
//               {
//                   points.push_back(p);
//               }
           }
        }
    }

    return depth-1;
}

void HilbertStrategy::RotatePointOrderBy90(std::vector<TooN::Vector<3> > & list, bool clockwise)
{
    if(clockwise)
    {
        for(unsigned int i=0; i<list.size(); i++)
        {
            TooN::Vector<3> t = list[i];
            list[i][0] = t[1];
            list[i][1] = -t[0];
        }
    }

    if(!clockwise)
    {
        //int s = sqrt(list.size());
        for(unsigned int i=0; i<list.size(); i++)
        {
            TooN::Vector<3> t = list[i];
            list[i][0] = -t[1];
            list[i][1] = t[0];
        }
    }

}

CNode * HilbertStrategy::findNode(int x, int y, std::vector<CNode*> &list)
{
    for(unsigned int i=0; i < list.size(); i++)
    {
        if(list[i]->grd_x == x && list[i]->grd_y == y)
            return list[i];
    }

    return NULL;
}

HilbertStrategy::~HilbertStrategy()
{

}

bool HilbertStrategy::UpdateIterator()
{
    bool flag = false;
    //** Initialization
    static bool firstCall = true;
    if(firstCall)
    {
        it = hilbert[LML].begin();
        curDepth = LML;
        //it = hilbert[0].begin();
        //curDepth = 0;
        firstCall = false;

        return true;
    }

    if(!lmWps.empty())
    {
        for(unsigned int i=0; i<hilbert[lastDepth].size(); i++)
            if(hilbert[lastDepth][i] == lmWps[0])
            {
                curDepth = lastDepth;
                it = hilbert[lastDepth].begin()+i;
                lmWps.erase(lmWps.begin());
                break;
            }

        return true;
    }

    while(!flag)
    {
        if(curDepth > 1 && !(*it)->IsNodeInteresting() /*&& !(*it)->parent->visited*/)
        {            
            bool stayAtBottom = false;
            if(false && curDepth == lastDepth)
            {
                for(unsigned int i=0; i<hilbert[curDepth].size(); i++)
                {
                    if(it == hilbert[curDepth].begin()+i)
                    {
                        if(i%4 !=0)
                        {
                            stayAtBottom = true;
                            break;
                        }
                    }
                }
            }

            if(!stayAtBottom)
            {
                CNode* parent = (*it)->parent;
                for(unsigned int i=0; i<hilbert[curDepth-1].size(); i++)
                {
                    if(hilbert[curDepth-1][i]==parent)
                    {
                        it = hilbert[curDepth-1].begin()+i;
                        curDepth--;
                        flag = true;
                        break;
                    }
                }
            }

        }
        else if(curDepth < LML && (*it)->IsNodeInteresting() && (*it)->ChildrenNeedVisitation())
        {
             /* ChildrenNeedVisitation() avoids this situation: when it goes up and finds out that the remaining children are uninteresting and
              * only some of the visited children are interesting.
              */

            //ROS_INFO("DOWN");
            CNode* parent = (*it);
            for(unsigned int i=0; i<hilbert[curDepth].size(); i++)
            {
                if(hilbert[curDepth][i]==parent)
                {
                    it = hilbert[curDepth+1].begin()+i*4;
                    curDepth++;
                    flag = true;
                    break;
                }
            }
        }

        CNode * cur_parent = (*it)->parent;
        //CNode * cur_node = (*it);
        int nseek = 0;
        while(!(*it)->NeedsVisitation()/*(*it)->visited || ((*it)->IsInterestingnessSet() && !(*it)->IsNodeInteresting())*/)
        {
            nseek++;
            it++;
            if(it == hilbert[curDepth].end())
            {
                return false;
            }
        }

        if(nseek > 1 && cur_parent != (*it)->parent)
        {
            for(unsigned int i=0; i<hilbert[curDepth-1].size(); i++)
            {
                if(hilbert[curDepth-1][i]==cur_parent)
                {
                    it = hilbert[curDepth-1].begin()+i;
                    curDepth--;
                    break;
                }
            }
            flag = false;
        }
        else
        {

            flag = true;
        }
    }


//    if((*it)->depth == 2)
//    {
//        GenerateLawnmower((*it), lmWps);
//        ROS_INFO("Lawnmower ..... %d ", lmWps.size());
//        return UpdateIterator();
//    }


    return true;
}

CNode* HilbertStrategy::GetNextNode()
{
    if(lms==NULL && curDepth == LML && (*it)->IsNodeInteresting() && (*it)->ChildrenNeedVisitation())
    {
        lms = new LawnmowerStrategy((*it), GetFirstLMNode(*it), GetLastLMNode(*it));
    }

    if(lms)
    {
        CNode * n = lms->GetNextNode();
        while(n!=NULL && !n->NeedsVisitation())
            n = lms->GetNextNode();

        if(n)
        {
            return n;
        }
        else
        {
            delete lms;
            lms = NULL;
        }
    }

    bool over = !UpdateIterator();
    if(over != isover)
    {
        for(unsigned int i=0; i< MAX_HILBERT_ORDER; i++)
        {
            if(waypointCount[i] > 0)
                ROS_INFO("#waypoints in Hilbert order %d:\t%d", i, waypointCount[i]);
        }
    }

    isover = over;
    if(!isover)
    {
        waypointCount[curDepth]++;
        nodeStack.push_back(*it);
        return *it;
    }

    return NULL;
}
void HilbertStrategy::hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey)
{
    if(key[']'])
    {
        curCurve = (curCurve+1)%10;
       // updateKey = false;
    }
    else if(key['['])
    {
        curCurve = (curCurve-1)%10;
        //updateKey = false;
    }

}

void HilbertStrategy::glDraw()
{

    for(int k=1; k< MAX_HILBERT_ORDER; k++)
    {
//        if(!isover)
//        {
//            if(k==curDepth && !hilbert[k].empty())
//            {
//                glColor3f(0.5,0.6,0.6);
//                glLineWidth(4);
//                glBegin(GL_LINES);
//                for(unsigned int i=0; i<hilbert[k].size()-1;i++)
//                {
//                    TooN::Vector<3> p1 =hilbert[k][i+1]->GetMAVWaypoint();
//                    TooN::Vector<3> p2 =hilbert[k][i]->GetMAVWaypoint();
//                    glVertex3f(p1[0],p1[1],p1[2]);
//                    glVertex3f(p2[0],p2[1],p2[2]);
//                }
//                glEnd();
//            }
//        }

        if(k==curCurve && !hilbert[k].empty())
        {
            glColor3f(1,0.5,0.0);
            glLineWidth(7);
            glBegin(GL_LINES);
            for(unsigned int i=0; i<hilbert[k].size()-1;i++)
            {
                TooN::Vector<3> p1 =hilbert[k][i+1]->GetMAVWaypoint();
                TooN::Vector<3> p2 =hilbert[k][i]->GetMAVWaypoint();
                glVertex3f(p1[0],p1[1],p1[2]);
                glVertex3f(p2[0],p2[1],p2[2]);
            }
            glEnd();

            glColor3f(0,0.0,0.0);
            glPointSize(10);
            glBegin(GL_POINTS);
            for(unsigned int i=0; i<hilbert[k].size();i++)
            {
                TooN::Vector<3> p2 =hilbert[k][i]->GetMAVWaypoint();
                glVertex3f(p2[0],p2[1],p2[2]);
            }
            glEnd();
        }
    }
}


void HilbertStrategy::GenerateLawnmower(CNode *parentNode, std::vector<CNode*>& lm)
{
    ROS_INFO("LM: start");

    lm.clear();

    CNode * firstN = GetFirstLMNode(parentNode);
    CNode * lastN  = GetLastLMNode(parentNode);
    ROS_INFO("LM: middle");
    //double eps = 0.2;
    //bool verlm = false;
    double l = fabs(firstN->footPrint[0]-firstN->footPrint[2]);

//    if(fabs(firstN->pos[0]-lastN->pos[0]) < eps) // decide if it is a vertical or horizontal lawnmower
//    {
//        verlm = true;
//    }

    Vector<2> interlapVec = makeVector(lastN->pos[0]-firstN->pos[0], lastN->pos[1]-firstN->pos[1]);
    normalize(interlapVec);
    Vector<2> testVec = makeVector(parentNode->pos[0]-lastN->pos[0], parentNode->pos[1]-lastN->pos[1]);
    normalize(testVec);
    Vector<2> interWp = makeVector(-interlapVec[1], interlapVec[0]);
    if( interWp*testVec < 0)
        interWp = makeVector(interlapVec[1], -interlapVec[0]);

    int n = floor((fabs(parentNode->footPrint[0]-parentNode->footPrint[2])+0.2)/l);

    Vector<3> interlapVec3 = l * makeVector(interlapVec[0], interlapVec[1], 0);
    Vector<3> interWp3 = l * makeVector(interWp[0], interWp[1], 0);

    ROS_INFO("LM: area:%.1f fp:%.1f", fabs(parentNode->footPrint[0]-parentNode->footPrint[2]), l);
    ROS_INFO("interlap: %f %f %f", interlapVec3[0], interlapVec3[1], interlapVec3[2]);
    ROS_INFO("interwp: %f %f %f", interWp3[0], interWp3[1], interWp3[2]);

    for(int i=0; i< n; i++)
        for(int j=0; j< n; j++)
        {
            int jj = (i%2 == 0)?j:n-j-1;
            Vector<3> npos = firstN->pos + i * interlapVec3 + jj * interWp3;
            lm.push_back(parentNode->GetNearestLeaf(npos));
        }

    ROS_INFO("LM: end");
}

CNode * HilbertStrategy::GetFirstLMNode(CNode *node)
{
    if(node->IsLeaf())
        return node;

    int parent_i=-1;
    for(unsigned int i=0; i<hilbert[node->depth].size(); i++)
    {
        if(hilbert[node->depth][i] == node)
        {
            parent_i = i;
            break;
        }
    }

    CNode* curnode = hilbert[node->depth+1][parent_i*4];
    parent_i = parent_i*4;

    while(curnode->depth < lastDepth)
    {
        curnode = hilbert[curnode->depth+1][parent_i*4];
        parent_i = parent_i*4;
    }

    return curnode;
}

CNode * HilbertStrategy::GetLastLMNode(CNode *node)
{
    if(node->IsLeaf())
        return node;

    int parent_i=-1;
    for(unsigned int i=0; i<hilbert[node->depth].size(); i++)
    {
        if(hilbert[node->depth][i] == node)
        {
            parent_i = i;
            break;
        }
    }

    CNode* curnode = hilbert[node->depth+1][parent_i*4+3];
    parent_i = parent_i*4+3;

    while(curnode->depth < lastDepth)
    {
        curnode = hilbert[curnode->depth+1][parent_i*4+3];
        parent_i = parent_i*4+3;
    }

    return curnode;
}


