#include "TargetPolygon.h"
#include <algorithm>
#include <GL/glut.h>
#include "ros/ros.h"
#include "NUCParam.h"

#define ANGLE(a,b,c) (acos( ((a-b)*(c-b)) / (sqrt((a-b)*(a-b))*sqrt((c-b)*(c-b))) ))
#define MIN(a,b) ((a<b)?a:b)
#define MAX(a,b) ((a<b)?b:a)
#define D2(a,b) (a-b)*(a-b)

double TargetPolygon::cellW=0;
using namespace std;

TargetPolygon::TargetPolygon()
{
 //   boundaryFlags = 0;
    SetBoundaryFlag(ALL, false);
    isLine = false;
    pc = makeVector(1,1,1);
    visited = false;
    ignored = false;
}

TargetPolygon::TargetPolygon(vector<CNode *> &cs, CNode *parentNode)
{
    SetBoundaryFlag(ALL, false);
    pc = makeVector(1,1,1);

    ignored = false;
    isLine = false;
    visited = false;
    if(parentNode)
    {
        parentSearchNodes.insert(parentNode);
        parentNode->targets.push_back(this);
    }

    std::copy(cs.begin(), cs.end(), std::back_inserter(cells));
    ProcessPolygon();
}

TargetPolygon::~TargetPolygon()
{
}

void TargetPolygon::SetBoundaryFlag(SIDE side, bool val)
{
    if(side != ALL)
    {
        boundaryFLags[(int)side] = val;
    }
    else if(side == ALL)
    {
        boundaryFLags[0] = val;
        boundaryFLags[1] = val;
        boundaryFLags[2] = val;
        boundaryFLags[3] = val;

    }
}
void TargetPolygon::OrBoundaryFlag(SIDE side, bool val)
{

    if(side != ALL)
    {
       boundaryFLags[(int)side] |= val;
    }
    else if(side == ALL)
    {
        boundaryFLags[0] |= val;
        boundaryFLags[1] |= val;
        boundaryFLags[2] |= val;
        boundaryFLags[3] |= val;

    }
}

bool TargetPolygon::GetBoundaryFlag(SIDE side) const
{
    return boundaryFLags[(int)side];
}

bool TargetPolygon::IsNonBoundaryTarget() const
{
    return !(boundaryFLags[0]||boundaryFLags[1]||boundaryFLags[2]||boundaryFLags[3]);
}

bool TargetPolygon::IsInside(const CNode *cell) const
{
    for(size_t i=0; i<cells.size(); i++)
        if(cells[i] == cell)
            return true;

    return false;
}

void TargetPolygon::ProcessPolygon()
{
    label = cells[0]->label;
    height = 0;
    base_idx[0] = -1;
    base_idx[1] = -1;

    Vector<4> fp = cells.back()->footPrint;
    cellW = fp[2]-fp[0];

    ConvexHull();
    FindBaseEdge();
    PlanLawnmower();
    SetLawnmowerHeight();
}

bool TargetPolygon::IsNeighbour(TargetPolygon *tp)
{
    for(set<CNode*>::iterator i=parentSearchNodes.begin(); i!=parentSearchNodes.end(); i++)
        for(set<CNode*>::iterator j=tp->parentSearchNodes.begin(); j!=tp->parentSearchNodes.end(); j++)
            if((*i)->IsNeighbour(*j))
                return true;

    return false;
}

void TargetPolygon::AddPolygon(TargetPolygon *p, bool changeLabels, bool process)
{
    parentSearchNodes.insert(p->parentSearchNodes.begin(), p->parentSearchNodes.end());

    for(int i=0; i<ALL; i++)
        boundaryFLags[i] = boundaryFLags[i] || p->boundaryFLags[i];

    for(size_t i=0; i < p->cells.size(); i++)
    {
        if(changeLabels)
            p->cells[i]->label = label;

        cells.push_back(p->cells[i]);
    }

    if(process)
        ProcessPolygon();
}

double TargetPolygon::GetTargetRegionsArea() const
{
    return ((double)cells.size())*cellW*cellW;
}

void TargetPolygon::GetCells(vector<CNode *> &v, const CNode* ofParent) const
{
    if(!ofParent)
        copy(cells.begin(), cells.end(), back_inserter(v));
    else
    {
        for(size_t i=0; i<cells.size(); i++)
            if(cells[i]->searchParentNode == ofParent)
            {
                v.push_back(cells[i]);
            }
    }
}

void TargetPolygon::GetLawnmowerPlan(vector<Vector<3> >  & v) const
{
    if(!lm.empty())
        std::copy(lm.begin(), lm.end(), std::back_inserter(v));
}

void TargetPolygon::MarkAsVisited()
{
    vector<CNode*>::iterator it = cells.begin();
    while(it != cells.end())
    {
        (*it++)->SetTreeVisited(true);
    }
}

void TargetPolygon::UpdateIsVisited()
{
    vector<CNode*>::iterator it = cells.begin();
    while(it != cells.end())
    {
        if((*it)->visited)
        {
            SetVisited(true);
            MarkIgnored();
            break;
        }
        ++it;
    }
}

void TargetPolygon::MarkIgnored()
{
    ignored = true;
}


Vector<3> TargetPolygon::GetMiddlePos()
{
    Vector<3> m = makeVector(0,0,0);

    if(ch.empty())
        return m;

    for(size_t i =0; i < ch.size(); i++)
        m += ch[i]->GetMAVWaypoint();

    return (1.0/m.size())*m;
}

void TargetPolygon::ReverseLawnmower()
{
    reverse(lm.begin(), lm.end());
}

void TargetPolygon::ConvexHull()
{
    ch.clear();
    // first node is the bottom most node
    unsigned first=0;
    for(unsigned int i=1; i<cells.size(); i++)
        if(cells[first]->pos[1]> cells[i]->pos[1])
            first = i;

    ch.push_back(cells[first]);
    //v.erase(v.begin()+first);

    center = cells[0]->pos;

    if(cells.size()<=1)
        return;

    //second node
    unsigned second=1;
    Vector<3> p1 = ch[0]->pos - makeVector(-1, 0, 0);
    Vector<3> p2 = ch.back()->pos;

    double ang = ANGLE(p1, p2, cells[1]->pos);
    for(unsigned int i=2; i<cells.size(); i++)
    {
        double a = ANGLE(p1, p2, cells[i]->pos);
        if(a> ang)
        {
            ang = a;
            second = i;
        }
    }

    ch.push_back(cells[second]);

    if(cells.size() <=2)
        return;

    //check if the cells form a line a line
    isLine = true;
    Vector<3> p_0 = cells[0]->pos;
    Vector<3> p_1 = cells[1]->pos;

    for(size_t i=2; i < cells.size() && isLine; i++)
    {
        Vector<3> p_n = cells[i]->pos;
        if(fabs((p_0[1]-p_1[1])*(p_0[0]-p_n[0]) - (p_0[1]-p_n[1])*(p_0[0]-p_1[0])) > 0.01)
        {
            isLine = false;
        }
    }

    if(isLine)
    {
        ch.clear();
        std::copy(cells.begin(), cells.end(), std::back_inserter(ch));
    }
    else
    {
        // the rest of the nodes
        while(true)
        {
            //if(n++ > 100) return;
            p1 = ch[ch.size()-2]->pos;
            p2 = ch.back()->pos;
            int next = -1;
            ang = -1;
            for(unsigned int i=0; i<cells.size(); i++)
            {
                if(cells[i] == ch.back())
                    continue;

                double a = ANGLE(p1, p2, cells[i]->pos);
                if(a> ang)
                {
                    ang = a;
                    next = i;
                }
            }

            if(next > -1)
            {
                if(std::find(ch.begin(), ch.end(), cells[next]) != ch.end())
                    break;
                else
                    ch.push_back(cells[next]);
            }
            else
            {
                break;
            }
        }

        // remove extra nodes (colinear edges)
        int sz = ch.size();

        for(int i=0; i < (int)ch.size(); i++)
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


    center = makeVector(0,0,0);
    for(size_t i=0; i < ch.size(); i++)
    {
        center += ch[i]->pos;
    }

    center = (1.0/(float)ch.size())*center;
}

void TargetPolygon::FindBaseEdge()
{
    base_idx[0] = -1;
    base_idx[1] = -1;

    if(isLine)
    {
        double maxDist = 0;
        for(size_t i=0; i< ch.size(); i++)
            for(size_t j=0; j< ch.size(); j++)
            {
                if(i==j) continue;
                double dist = (ch[i]->pos - ch[j]->pos)*(ch[i]->pos - ch[j]->pos);
                if(dist > maxDist)
                {
                    maxDist = dist;
                    base_idx[0] = i;
                    base_idx[1] = j;
                }
            }

        return;
    }

    int sz = ch.size();
    double minHeight = 999999;
    int min_idx = -1;

    if(sz == 0)
    {
        return;
    }
    else if(sz == 1)
    {
        base_idx[0] = 0;
        base_idx[1] = 0;
        return;
    }
    else if(sz == 2)
    {
        base_idx[0] = 0;
        base_idx[1] = 1;
        return;
    }

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
            height = minHeight;
            min_idx = i;
        }
    }

    double sd = pointToLineSignedDist(ch[min_idx]->GetMAVWaypoint(), ch[(min_idx+1)%sz]->GetMAVWaypoint(), ch[(min_idx+2)%sz]->GetMAVWaypoint());
    if(sd > 0)
    {
        base_idx[0] = min_idx;
        base_idx[1] = (min_idx+1)%sz;
        //return std::pair<int,int>(min_idx, (min_idx+1)%sz);
    }
    else
    {
        base_idx[1] = min_idx;
        base_idx[0] = (min_idx+1)%sz;
        //return std::pair<int,int>((min_idx+1)%sz, min_idx);
    }
}

double TargetPolygon::pointToLineDist(Vector<3> p1, Vector<3> p2, Vector<3> x) const
{
    double d = fabs((p2[0]-p1[0])*(p1[1]-x[1]) - (p1[0]-x[0])*(p2[1]-p1[1]))/sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1]));
    return d;
}

double TargetPolygon::pointToLineSignedDist(Vector<3> p1, Vector<3> p2, Vector<3> x) const
{
    double d = ((p2[0]-p1[0])*(p1[1]-x[1]) - (p1[0]-x[0])*(p2[1]-p1[1]))/sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1]));
    return d;
}

bool TargetPolygon::GetLineSegmentIntersection(Vector<3> p0, Vector<3> p1, Vector<3> p2,
                                               Vector<3> p3, Vector<3> &intersection_p) const
{
    float p0_x = p0[0], p0_y = p0[1], p1_x = p1[0], p1_y=p1[1], p2_x = p2[0], p2_y=p2[1], p3_x = p3[0], p3_y=p3[1];

    float s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    float s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
        // Collision detected
        //if (i_x != NULL)
        intersection_p[0] = p0_x + (t * s1_x);
        intersection_p[1] = p0_y + (t * s1_y);
        return true;
    }

    return false; // No collision

}

void TargetPolygon::SetLawnmowerHeight()
{
    double interlap_d = NUCParam::high_res_cells*cellW;
    double hi_res_lm_height = (0.5*interlap_d)/tan(0.5*NUCParam::FOV);

    for(size_t i =0; i<lm.size(); i++)
    {
        lm[i][2] = hi_res_lm_height;
    }
}

void TargetPolygon::PlanLawnmower()
{
    lm.clear();

    double interlap_d = NUCParam::high_res_cells*cellW;

    if(ch.size() == 0)
    {
        return;
    }
    else if(ch.size()==1 && base_idx[0] ==0 && base_idx[1]==0)
    {
        Vector<3> p_tmp = ch[0]->GetMAVWaypoint();
        lm.push_back(p_tmp + (cellW/2)*makeVector(-1,0,0));
        lm.push_back(p_tmp + (cellW/2)*makeVector(1,0,0));
        return ;
    }
    else if(ch.size()==2 && base_idx[0] ==0 && base_idx[1]==1)
    {
        Vector<3> p1_tmp = ch[0]->GetMAVWaypoint();
        Vector<3> p2_tmp = ch[1]->GetMAVWaypoint();

        lm.push_back(p1_tmp);
        lm.push_back(p2_tmp);
        return ;
    }

    if(isLine)
    {
        Vector<3> p1_tmp = ch[base_idx[0]]->GetMAVWaypoint();
        Vector<3> p2_tmp = ch[base_idx[1]]->GetMAVWaypoint();

        lm.push_back(p1_tmp);
        lm.push_back(p2_tmp);
        return ;
    }

    Vector<3> baseDir  = ch[base_idx[1]]->GetMAVWaypoint() - ch[base_idx[0]]->GetMAVWaypoint();
    Vector<3> baseDirNorm = baseDir;
    baseDirNorm[2] = 0;
    normalize(baseDirNorm);

    //clockwise orthogonal vector
    Vector<3> sweepDir = makeVector(baseDir[1], -baseDir[0], 0);
    normalize(sweepDir);

    Vector<3> sn0 = ch[base_idx[0]]->GetMAVWaypoint();
    Vector<3> en0 = ch[base_idx[1]]->GetMAVWaypoint();
    double offset0 = interlap_d*0.5;

    sn0 += (offset0-cellW/2) * sweepDir;
    en0 += (offset0-cellW/2) * sweepDir;

    // first lm track
    lm.push_back(sn0);
    lm.push_back(en0);

    double n =-1;
    while(height / interlap_d > n)
    {
//        if(n > 300)
//            break;

        n+=1.0;

        Vector<3> sn = sn0;
        Vector<3> en = en0;
        sn += n * interlap_d * sweepDir;
        en += n * interlap_d * sweepDir;

        sn -=  2 * NUCParam::area_length * baseDirNorm;
        en +=  2 * NUCParam::area_length * baseDirNorm;

        vector<Vector<3> > intersections;

        for(size_t i=0; i< ch.size(); i++)
        {
            Vector<3> ise =makeVector(0,0, sn0[2]);
            if(GetLineSegmentIntersection(sn, en, ch[i]->GetMAVWaypoint(), ch[(i+1)%(ch.size())]->GetMAVWaypoint(), ise))
            {
                intersections.push_back(ise);
            }
        }

        if(intersections.size()<=1)
        {
           //ROS_INFO("No lawnmower track found ...");
           //break;
        }
        else
        {
            if(intersections.size() > 2)
            {
                for(size_t i=0; i<intersections.size(); i++)
                {
                    for(size_t j=i+1; j<intersections.size(); j++)
                    {
                        if(D2(intersections[i],intersections[j]) < 0.1)
                        {
                            intersections.erase(intersections.begin()+j);
                            break;
                        }
                    }
                }
            }

            if(intersections.size()>=2)
            {
                if(D2(intersections[0],lm[lm.size()-2]) < D2(intersections[1],lm[lm.size()-2]))
                {
                    lm.push_back(intersections[1]);
                    lm.push_back(intersections[0]);
                }
                else
                {
                    lm.push_back(intersections[0]);
                    lm.push_back(intersections[1]);
                }
            }
        }

      }

    if(lm.size() > 2)
    {
        lm.erase(lm.begin());
        lm.erase(lm.begin());
    }
    else
    {
        lm[0] -= ((offset0-cellW/2)-height/2) * sweepDir;
        lm[1] -= ((offset0-cellW/2)-height/2) * sweepDir;
    }

 }

void TargetPolygon::glDraw()
{    
    if(ignored)
        return;

//    glLineWidth(4);
//    glColor3f(0,0,1);
//    glBegin(GL_LINES);
//    for(set<CNode*>::iterator i=parentSearchNodes.begin(); i!=parentSearchNodes.end(); i++)
//    {
//        Vector<3> c = GetCenter();
//        Vector<3> pcn = (*i)->GetMAVWaypoint();
//        glVertex3f(c[0], c[1], c[2]);
//        glVertex3f(pcn[0], pcn[1], pcn[2]);

//    }
//    glEnd();

    //if(ch.size()<=1)
    //    return;

    //glColor3f(ch[0]->colorBasis[0], ch[0]->colorBasis[1], ch[0]->colorBasis[2]);
//    glColor3f(pc[0], pc[1], pc[2]);
//    glLineWidth(4);
//    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//    glBegin(GL_POLYGON);
//    for(unsigned int i=0; i<ch.size();i++)
//    {
//        //glColor3f(RAND((i%5)/0.5,(i%5)/0.5+0.2),RAND(0,1),RAND(0,1));
//        TooN::Vector<3> p1 = ch[i]->GetMAVWaypoint();
//        glVertex3f(p1[0],p1[1],p1[2]);
//    }
//    glEnd();

//    if(base_idx[0] != -1 && base_idx[1]!=-1)
//    {
//        glColor3f(1,0,0);
//        glPointSize(10);
//        glBegin(GL_POINTS);
//        TooN::Vector<3> p1 = ch[base_idx[0]]->GetMAVWaypoint();
//        TooN::Vector<3> p2 = ch[base_idx[1]]->GetMAVWaypoint();
//        glColor3f(1,0,0);
//        glVertex3f(p1[0],p1[1],p1[2]);
//        glColor3f(0,1,0);
//        glVertex3f(p2[0],p2[1],p2[2]);
//        glEnd();

//        if(lm.size() > 1)
//        {
//            glColor4f(0.5,0.5,1, (visited?1.0:0.4));
//            glLineWidth(visited?7:4);
//            glBegin(GL_LINES);
//            //glPointSize(8);
//            //glBegin(GL_POINTS);
//            for(size_t i=0; i+1<lm.size(); i+=1)
//            {
//               TooN::Vector<3> p1 = lm[i];
//               TooN::Vector<3> p2 = lm[i+1];
//               glVertex3f(p1[0], p1[1], p1[2]+0.5);
//               glVertex3f(p2[0], p2[1], p2[2]+0.5);
//            }
//            glEnd();
//        }
//    }

    for(int i=0; i <4; i++)
    {
        if(boundaryFLags[i])
        {
            double dx = (i==0)?-3:((i==1)?3:0);
            double dy = (i==2)? 3:((i==3)?-3:0);

            glLineWidth(5);
            glBegin(GL_LINES);
            glColor3f(1,1,0);
            glVertex3f(center[0], center[1], center[2]);
            glColor3f(0,1,1);
            glVertex3f(center[0]+dx , center[1]+dy, center[2]);
            glEnd();
        }
    }
}
