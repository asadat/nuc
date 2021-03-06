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

    CNode::PopulateInt_Thr(lastDepth);
    root->PrintoutParams();
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

    if(depth>MAX_HILBERT_ORDER)
        depth = MAX_HILBERT_ORDER;


    ROS_INFO("depth: %d", depth);

    hilbert[0].push_back(parent);

    hilbert[1].push_back(findNode(0,1,parent->children));
    hilbert[1].push_back(findNode(0,0,parent->children));
    hilbert[1].push_back(findNode(1,0,parent->children));
    hilbert[1].push_back(findNode(1,1,parent->children));

    copy(hilbert[1].begin(), hilbert[1].end(), std::back_inserter(hilbert[0][0]->ordered_children));

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

               CNode * child_hilb_ord = parent->GetNearestNode(p);
               hilbert[lvl].push_back(child_hilb_ord);
               child_hilb_ord->parent->ordered_children.push_back(child_hilb_ord);

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
    //static bool first = true;

//    if(first && clockwise)
//    {
//        for(unsigned int i=0; i<list.size(); i++)
//        {
//            ROS_INFO("O(%f,%f)", list[i][0],list[i][1]);
//        }
//    }
    if(clockwise)
    {
        //int s = sqrt(list.size());
        for(unsigned int i=0; i<list.size(); i++)
        {
            TooN::Vector<3> t = list[i];
            list[i][0] = t[1];
            list[i][1] = -t[0];
        }

//        if(first)
//        {
//            for(unsigned int i=0; i<list.size(); i++)
//            {
//                ROS_INFO("M(%f,%f)", list[i][0],list[i][1]);
//            }

//            first = false;

//        }
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
        //start at leaf level
        it = hilbert[lastDepth].begin();
        curDepth = lastDepth;

        //start at the root
        //it = hilbert[0].begin();
        //curDepth = 0;

        firstCall = false;
        return true;
    }
    //**

    while(!flag)
    {
        if(curDepth > 1 && !(*it)->IsNodeInteresting())
        {            
            CNode* parent = (*it)->parent;
            if(!parent->visited/*!parent->ChildrenNeedVisitation()*/)
            {
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
        else if(curDepth < lastDepth && (*it)->IsNodeInteresting() && (*it)->ChildrenNeedVisitation())
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
        while(!(*it)->NeedsVisitation()  && nseek < 4/*(*it)->visited || ((*it)->IsInterestingnessSet() && !(*it)->IsNodeInteresting())*/)
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

    return true;
}

CNode* HilbertStrategy::GetNextNode()
{
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
