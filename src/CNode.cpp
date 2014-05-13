#include "CNode.h"
#include <math.h>
#include <GL/glut.h>
#include "NUCParam.h"

#define IN(x,y)    (y[0] <= x[0] && x[0] <= y[2] && y[1] <= x[1] && x[1] <= y[3])


//float CNode::fov = 90 *3.14/(180);
int CNode::bf_sqrt = 2;
//float CNode::minFootprintWidth = 3;


CNode::CNode(Rect target_foot_print):parent(NULL)
{
    visited = false;
    grd_x = 0;
    grd_y = 0;
    depth = 0;
    isInteresting = false;
    trueIsInteresting = false;
    footPrint = target_foot_print;
    pos[0] = (0.5)*(footPrint[0]+footPrint[2]);
    pos[1] = (0.5)*(footPrint[1]+footPrint[3]);
    pos[2] = (0.5)*fabs((footPrint[0]-footPrint[2])/tan(NUCParam::FOV/2.0));

    PopulateChildren();
}

CNode::~CNode()
{

}

CNode * CNode::CreateChildNode(Rect fp)
{
    CNode* cnode = new CNode(fp);
    cnode->parent = this;
    cnode->depth = depth +1;
    children.push_back(cnode);
    return cnode;
}

void CNode::PopulateChildren()
{
    double fps = (footPrint[2]-footPrint[0]);
    if(fps <= NUCParam::min_footprint)
    {
        ROS_INFO_ONCE("leaf: fp:%f height:%f", fps, pos[2]);
        return;
    }

    double dl = fps/bf_sqrt;

    for(int i=0; i<bf_sqrt; i++)
        for(int j=0; j<bf_sqrt; j++)
        {
            Rect fp;
            fp[0] = footPrint[0]+i*dl;
            fp[1] = footPrint[1]+j*dl;
            fp[2] = fp[0]+dl;
            fp[3] = fp[1]+dl;

            CNode* ch = CreateChildNode(fp);
            ch->grd_x = i;
            ch->grd_y = j;

        }
}

TooN::Vector<3> CNode::GetMAVWaypoint()
{
    TooN::Vector<2> c = TooN::makeVector(NUCParam::cx, NUCParam::cy);
    TooN::Matrix<2,2,double> rot = TooN::Data(cos(NUCParam::area_rotation*D2R), sin(NUCParam::area_rotation*D2R),
                                     -sin(NUCParam::area_rotation*D2R), cos(NUCParam::area_rotation*D2R));
    TooN::Vector<2> v = TooN::makeVector(pos[0],pos[1]);
    v = c+rot*(v-c);

    return TooN::makeVector(v[0], v[1], pos[2]);
}

TooN::Vector<2> CNode::Rotation2D(TooN::Vector<2> v, double deg, TooN::Vector<2> c)
{
    TooN::Matrix<2,2,double> rot = TooN::Data(cos(deg*D2R), sin(deg*D2R),
                                     -sin(deg*D2R), cos(deg*D2R));



    return (c+ rot*(v-c));
}

TooN::Vector<3> CNode::Rotation2D(TooN::Vector<3> v, double deg, TooN::Vector<2> c)
{
    TooN::Matrix<2,2,double> rot = TooN::Data(cos(deg*D2R), sin(deg*D2R),
                                     -sin(deg*D2R), cos(deg*D2R));

    TooN::Vector<2> v2 = TooN::makeVector(v[0],v[1]);
    v2 = c+rot*(v2-c);

    return TooN::makeVector(v2[0], v2[1], v[2]);
}

void CNode::glDraw()
{
//    TooN::Vector<2> c = TooN::makeVector(NUCParam::cx, NUCParam::cy);
//    TooN::Matrix<2,2,double> rot = TooN::Data(cos(NUCParam::area_rotation*D2R), sin(NUCParam::area_rotation*D2R),
//                                     -sin(NUCParam::area_rotation*D2R), cos(NUCParam::area_rotation*D2R));

    TooN::Vector<3> v2 = Rotation2D(pos, NUCParam::area_rotation, TooN::makeVector(NUCParam::cx, NUCParam::cy));
    //v2 = c+rot*(v2-c);

    if(parent != NULL)
    {
        glLineWidth(1);
        glColor3f(.2,.2,.2);
        glBegin(GL_LINES);
        TooN::Vector<3> v1 = Rotation2D(parent->pos, NUCParam::area_rotation, TooN::makeVector(NUCParam::cx, NUCParam::cy));

        glVertex3f(v1[0],v1[1],v1[2]);
        glVertex3f(v2[0],v2[1],v2[2]);
        glEnd();
    }

    if(IsNodeInteresting())
    {
        glPointSize(10);
        glColor3f(0,1,0);
    }
    else
    {
        glPointSize(3);
        glColor3f(0,0,0);
    }

    glBegin(GL_POINTS);
    glVertex3f(v2[0],v2[1],v2[2]);
    glEnd();

    for(unsigned int i=0; i<children.size(); i++)
        children[i]->glDraw();

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    if(children.empty())
    {
        TooN::Vector<2,double> p1,p2,p3,p4,v1,v2,v3,v4;
        p1[0] = footPrint[0];
        p1[1] = footPrint[1];
        p2[0] = footPrint[2];
        p2[1] = footPrint[3];

        p3 = TooN::makeVector(p1[0], p2[1]);
        p4 = TooN::makeVector(p2[0], p1[1]);

        v1 = Rotation2D(p1, NUCParam::area_rotation,TooN::makeVector(NUCParam::cx, NUCParam::cy));
        v2 = Rotation2D(p3, NUCParam::area_rotation,TooN::makeVector(NUCParam::cx, NUCParam::cy));
        v3 = Rotation2D(p2, NUCParam::area_rotation,TooN::makeVector(NUCParam::cx, NUCParam::cy));
        v4 = Rotation2D(p4, NUCParam::area_rotation,TooN::makeVector(NUCParam::cx, NUCParam::cy));

//        TooN::Vector<2,double> r1 = c + rot*(v1-c);
//        TooN::Vector<2,double> r2 = c + rot*(v2-c);
//        TooN::Vector<2,double> r3 = c + rot*(v3-c);
//        TooN::Vector<2,double> r4 = c + rot*(v4-c);

        glColor3f(0.95,1,0.95);
        glBegin(GL_POLYGON);
        glVertex3f(v1[0],v1[1], 0.1);
        glVertex3f(v2[0],v2[1], 0.1);
        glVertex3f(v3[0],v3[1], 0.1);
        glVertex3f(v4[0],v4[1], 0.1);
        glVertex3f(v1[0],v1[1], 0.1);
        //glVertex3f(footPrint[0],footPrint[3], 0.1);
        //glVertex3f(footPrint[2],footPrint[3], 0.1);
        //glVertex3f(footPrint[2],footPrint[1], 0.1);
        glEnd();
    }

}

void CNode::PropagateDepth()
{
    if(parent == NULL)
        depth = 0;
    else
        depth = parent->depth +1;

    for(unsigned int i=0; i<children.size(); i++)
    {
        children[i]->PropagateDepth();
    }
}

bool CNode::PropagateInterestingness(Rect r)
{
    if(children.empty())
    {
        if(trueIsInteresting)
            return true;

        trueIsInteresting = IN(pos,r);

        return trueIsInteresting;
    }

    bool flag = false;
    for(unsigned int i=0; i<children.size(); i++)
    {
        bool res = children[i]->PropagateInterestingness(r);
        flag  = flag || res;
    }

    trueIsInteresting = trueIsInteresting || flag;

    return trueIsInteresting;
}

bool CNode::VisitedInterestingDescendentExists()
{
    if(children.empty())
        return (visited && IsNodeInteresting());

    bool flag = false;
    for(unsigned int i=0; i<children.size(); i++)
    {
        flag = children[i]->VisitedInterestingDescendentExists();
        if(flag)
            return true;
    }

    return false;
}

CNode* CNode::GetNearestLeaf(TooN::Vector<3> p)
{
    if(children.empty())
        return this;

    int minidx=0;
    double minDist = 99999999999;
    for(unsigned int i=0; i<children.size(); i++)
    {
        double dist = (children[i]->pos-p)*(children[i]->pos-p);
        if(dist < minDist)
        {
            minidx = i;
            minDist = dist;
        }
    }

    return children[minidx]->GetNearestLeaf(p);
}

void CNode::GetNearestLeafAndParents(TooN::Vector<3> p, std::vector<CNode*> & list)
{
    if(children.empty())
        return;

    int minidx=0;
    double minDist = 99999999999;
    for(unsigned int i=0; i<children.size(); i++)
    {
        double dist = (children[i]->pos-p)*(children[i]->pos-p);
        if(dist < minDist)
        {
            minidx = i;
            minDist = dist;
        }
    }

    list.push_back(children[minidx]);
    return children[minidx]->GetNearestLeafAndParents(p, list);
}
