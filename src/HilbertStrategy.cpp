#include "HilbertStrategy.h"
#include "GL/glut.h"
#include <stdio.h>
#include "NUCParam.h"

using namespace TooN;

HilbertStrategy::HilbertStrategy(CNode *root)
{
    assert(NUCParam::bf_sqrt == 2);


    int d = HilbertCurveOther(root);
    for(unsigned int i=0; i<hilbert[d].size(); i++)
        nodeStack.push_back(hilbert[d][i]);
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

//int HilbertStrategy::HilbertCurve(CNode *parent)
//{

//    Rect r = parent->GetFootPrint();
//    double l = fabs(r[0]-r[2]);

//    int depth =1;
//    while(l > NUCParam::min_footprint)
//    {
//        depth++;
//        l /= NUCParam::bf_sqrt;
//    }

//    if(depth>99)
//        depth = 99;

//    ROS_INFO("depth: %d", depth);

//    hilbert[0].push_back(parent);

//    hilbert[1].push_back(findNode(0,1,parent->children));
//    hilbert[1].push_back(findNode(0,0,parent->children));
//    hilbert[1].push_back(findNode(1,0,parent->children));
//    hilbert[1].push_back(findNode(1,1,parent->children));

//    std::vector<Vector<2, int> > last_order;
//    last_order.push_back(makeVector(-1,1));
//    last_order.push_back(makeVector(-1,-1));
//    last_order.push_back(makeVector(1,-1));
//    last_order.push_back(makeVector(1,1));

//    for(int lvl=2; lvl < depth; lvl++)
//    {
//        std::vector<Vector<2, int> > cur_order;
//        for(int ii=0; ii<4; ii++)
//        {
//            std::vector<Vector<2, int> > order;
//            for(unsigned int jj=0; jj < last_order.size(); jj++)
//            {
//                Vector<2, int> idx;
//                idx[0] = last_order[jj][0];
//                idx[1] = last_order[jj][1];
////                if(ii==0 || ii==3)
////                    order.insert(order.begin(),idx);
////                else
//                    order.push_back(idx);
//            }

////            if(ii == 0)
////                RotateOrderBy90(order, false);

////            if(ii == 3)
////                RotateOrderBy90(order, true);

//            double s = sqrt(order.size());
//            double dx = (fabs(parent->footPrint[0]-parent->footPrint[2])*0.5)/s;
//            double dy = (fabs(parent->footPrint[1]-parent->footPrint[3])*0.5)/s;

//            double DX,DY;
//            if(ii == 0)
//            {    DX=-1; DY=1;}
//            else if(ii == 1)
//            {    DX=-1; DY=-1;}
//            else if(ii == 2)
//            {    DX=1; DY=-1;}
//            else if(ii == 3)
//            {    DX=1; DY=1;}

//            for(unsigned int jj=0; jj<order.size(); jj++)
//            {
//                Vector<3> p = makeVector(0,0,0);

//                p = parent->pos;
//                p[0] += DX*fabs(parent->footPrint[0]-parent->footPrint[2])*0.25;
//                p[1] += DY*fabs(parent->footPrint[1]-parent->footPrint[3])*0.25;

//                //p[0] -= fabs(parent->footPrint[0]-parent->footPrint[2])*0.25;
//                //p[1] -= fabs(parent->footPrint[1]-parent->footPrint[3])*0.25;

//                //p[0] += dx*0.5;
//                //p[1] += dy*0.5;

//                p[0] += (order[jj][0] < 0)?0.5*dx:-0.5*dx;
//                p[1] += (order[jj][1] < 0)?0.5*dy:-0.5*dy;

//                p[0] += dx * (double)(order[jj][0]);
//                p[1] += dy * (double)(order[jj][1]);

//                p[2] = (0.5)*fabs((dx)/tan(NUCParam::FOV/2.0));

//                hilbert[lvl].push_back(parent->GetNearestNode(p));
//                Vector<2, int> newidx = order[jj]+makeVector((s/2)*DX,(s/2)*DY);
//                if(newidx[0] == 0)
//                {
//                    newidx[0] = DX;
//                }

//                if(newidx[1] == 0)
//                {
//                    newidx[1] = DY;
//                }

//                cur_order.push_back(newidx);
//                if(lvl+1 == depth)
//                {
//                    points.push_back(p);
//                }
//            }
//        }

//        last_order.clear();
//        for(int i=0;i<cur_order.size();i++)
//            last_order.push_back(cur_order[i]);
//    }

//    return depth-1;

//}

//void HilbertStrategy::RotateOrderBy90(std::vector<TooN::Vector<2, int> > & list, bool clockwise)
//{
//    static bool first = true;

//    if(first && clockwise)
//    {
//        for(unsigned int i=0; i<list.size(); i++)
//        {
//            ROS_INFO("O(%d,%d)", list[i][0],list[i][1]);
//        }
//    }
//    if(clockwise)
//    {
//        int s = sqrt(list.size());
//        for(unsigned int i=0; i<list.size(); i++)
//        {
//            TooN::Vector<2, int> t = list[i];
//            list[i][0] = t[1];
//            list[i][1] = -t[0];

////            int tmp = list[i][0];
////            list[i][0] = list[i][1];
////            list[i][1] = tmp;
//            //list[i][1] = s-1 - list[i][1];
//        }

//        if(first)
//        {
//            for(unsigned int i=0; i<list.size(); i++)
//            {
//                ROS_INFO("M(%d,%d)", list[i][0],list[i][1]);
//            }

//            first = false;

//        }
//    }

//    if(!clockwise)
//    {
//        int s = sqrt(list.size());
//        for(unsigned int i=0; i<list.size(); i++)
//        {
//            TooN::Vector<2, int> t = list[i];
//            list[i][0] = -t[1];
//            list[i][1] = t[0];

////            int tmp = list[i][0];
////            list[i][0] = list[i][1];
////            list[i][1] = tmp;
//            //list[i][0] = s-1 - list[i][0];
//        }
//    }

//}


void HilbertStrategy::RotatePointOrderBy90(std::vector<TooN::Vector<3> > & list, bool clockwise)
{
    static bool first = true;

    if(first && clockwise)
    {
        for(unsigned int i=0; i<list.size(); i++)
        {
            ROS_INFO("O(%f,%f)", list[i][0],list[i][1]);
        }
    }
    if(clockwise)
    {
        //int s = sqrt(list.size());
        for(unsigned int i=0; i<list.size(); i++)
        {
            TooN::Vector<3> t = list[i];
            list[i][0] = t[1];
            list[i][1] = -t[0];
        }

        if(first)
        {
            for(unsigned int i=0; i<list.size(); i++)
            {
                ROS_INFO("M(%f,%f)", list[i][0],list[i][1]);
            }

            first = false;

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

CNode* HilbertStrategy::GetNextNode()
{
    if(nodeStack.empty())
        return NULL;

    CNode* node = nodeStack.front();
    nodeStack.erase(nodeStack.begin());
    return node;
}

void HilbertStrategy::glDraw()
{
    if(nodeStack.size() > 2)
    {
        glColor3f(0.5,0.6,0.6);
        glLineWidth(4);
        glBegin(GL_LINES);
        for(unsigned int i=0; i<nodeStack.size()-1;i++)
        {
            TooN::Vector<3> p1 =nodeStack[i+1]->GetMAVWaypoint();
            TooN::Vector<3> p2 =nodeStack[i]->GetMAVWaypoint();

            glVertex3f(p1[0],p1[1],p1[2]);
            glVertex3f(p2[0],p2[1],p2[2]);
        }
        glEnd();
    }

//    for(int k=1; k< 10; k++)
//    {
//        if(!hilbert[k].empty())
//        {
//            glColor3f(0.0,0.0,(0.2)*((double)k));
//            glLineWidth(3+0.5*k);
//            glBegin(GL_LINES);
//            for(unsigned int i=0; i<hilbert[k].size()-1;i++)
//            {
//                TooN::Vector<3> p1 =hilbert[k][i+1]->GetMAVWaypoint();
//                TooN::Vector<3> p2 =hilbert[k][i]->GetMAVWaypoint();
//                glVertex3f(p1[0],p1[1],p1[2]);
//                glVertex3f(p2[0],p2[1],p2[2]);
//            }
//            glEnd();
//        }
//    }

}
