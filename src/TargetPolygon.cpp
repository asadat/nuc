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
std::function<CNode*(int,int)> TargetPolygon::GetNode;

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

TargetPolygon::TargetPolygon(vector<CNode *> &cs, CNode *parentNode, std::function<CNode *(int, int)> gn)
{
    GetNode = gn;
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
    for(auto pn:parentSearchNodes)
    {
        auto it = find(pn->targets.begin(), pn->targets.end(), this);
        if(it != pn->targets.end())
        {
            pn->targets.erase(it);
        }
    }
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

    ROS_INFO("here 1");
    FindApproximatePolygon();
    ROS_INFO("here 2");
    ConvexHull();
    ROS_INFO("here 3");
    FindBaseEdge();
    ROS_INFO("here 4");
    PlanLawnmower();
    ROS_INFO("here 5");
    SetLawnmowerHeight();
    ROS_INFO("here 6");

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
    ROS_INFO("AT 0 ");
    parentSearchNodes.insert(p->parentSearchNodes.begin(), p->parentSearchNodes.end());
    ROS_INFO("AT 1 ");
    for(int i=0; i<ALL; i++)
        boundaryFLags[i] = boundaryFLags[i] || p->boundaryFLags[i];

    for(size_t i=0; i < p->cells.size(); i++)
    {
        if(changeLabels)
            p->cells[i]->label = label;

        cells.push_back(p->cells[i]);
    }
ROS_INFO("AT 2 ");
    if(process)
        ProcessPolygon();
ROS_INFO("AT 3 ");                }

double TargetPolygon::GetTargetRegionsArea() const
{
    return ((double)cells.size())*cellW*cellW;
}

double TargetPolygon::GetTrueTargetRegionsArea() const
{
    int n=0;
    for(const auto&c:cells)
        if(c->p_X >= c->cut_off)
            n++;

    return n*cellW*cellW;
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

int TargetPolygon::GetApproxNeighboursOfLabel(CNode* node, bool neighbour_8)
{
    int di = neighbour_8?1:2;
    int n=0;
    for(int i=0; i<8; i+=di)
    {
        auto nb = GetNeighbour_8(node, i);
        if(nb && nb->label==node->label && nb->approx_label==nb->label)
        {
            n++;
        }
    }

    return n;
}

void TargetPolygon::RemoveSkinnyPart(CNode* tip_node)
{
    if(!tip_node)
        return;

    if(tip_node->checked_for_skinny)
        return;

    tip_node->checked_for_skinny = true;
    for(int i=0; i<8; i+=2)
    {
        auto nb = GetNeighbour_8(tip_node, i);
        if(nb && nb->label == tip_node->label && nb->approx_label==nb->label)
        {
            if(GetApproxNeighboursOfLabel(nb, false) <= 1)
            {
                nb->approx_label = -1;
                RemoveSkinnyPart(nb);
            }
        }
    }
}

void TargetPolygon::FindApproximatePolygon()
{
    for(auto nd:cells)
    {
        nd->approx_label = nd->label;
        nd->checked_for_skinny = false;
    }

    // remove the parts with 1-cell width
    for(size_t i=0; i<cells.size(); i++)
    {
        int nbs=0;
        for(int j=0; j<8; j+=2)
        {
            auto c = GetNeighbour_8(cells[i], j);
            if(c && c->label == cells[i]->label)
            {
                nbs++;
            }
        }

        if(nbs <= 1)
        {
            cells[i]->approx_label = -1;
            RemoveSkinnyPart(cells[i]);
        }
    }

    set<CNode*> nds;
    for(auto &nd:cells)
    {
        if(nd->approx_label != nd->label)
            continue;

        for(int i=0; i<8; i+=2)
        {
            auto c = GetNeighbour_8(nd, i);

            if(!c || (c->label!=this->label) || (c->approx_label != nd->label))
            {
                nds.insert(nd);
                break;
            }
        }
    }

    if(nds.empty())
        return;

    copy(nds.begin(), nds.end(), back_inserter(boundaryNodes));

    auto minit = std::min_element(nds.begin(), nds.end(), [](CNode* x, CNode* y)
    {
        if(x->grd_y < y->grd_y) return true;
        if(x->grd_y > y->grd_y) return false;
        return (x->grd_x < y->grd_x);
    });

    approxPoly.clear();
    approxPoly.push_back(*minit);
    nds.erase(minit);

    while(!nds.empty())
    {
        CNode* node = approxPoly.back();
        size_t i=0;
        for(; i<8; i++)
        {
            auto nb = GetNeighbour_8(node, i);
            if(nb == NULL) continue;

            auto nb_it = nds.find(nb);
            if(nb_it != nds.end())
            {
                approxPoly.push_back(*nb_it);
                nds.erase(nb_it);
                break;
            }
        }

        if(i>=8)
        {
            //ROS_ERROR("incomplete approximate polygon!!!!!!!!!!!!! %d %d", node->grd_x, node->grd_y);
            break;
        }
    }

    vector<CNode*> simplified_approx_polygon;
    simplified_approx_polygon.push_back(approxPoly.front());
    for(auto it = approxPoly.begin(); it!= approxPoly.end();)
    {
        const double mean_dist_threshold = NUCParam::min_footprint*0.5;
        for(auto it_end = it+1; it_end != approxPoly.end(); it_end++)
        {
            auto it_next = it_end+1;
            if(it_next == approxPoly.end())
            {
                simplified_approx_polygon.push_back(*it_end);
                it = it_next;
                break;
            }
            else
            {
                double mean_dist = 0;
                double sz = 0;
                for(auto it_tmp=it+1; it_tmp!=it_next; it_tmp++)
                {
                    sz+=1;
                    mean_dist += pointToLineDist((*it)->GetMAVWaypoint(), (*it_next)->GetMAVWaypoint(), (*it_tmp)->GetMAVWaypoint());
                }

                if(sz < 1.0)
                    sz = 1.0;

                mean_dist /= sz;

                if(mean_dist > mean_dist_threshold)
                {
                    ROS_INFO("************ mean_dist: %f", mean_dist);
                    it = it_end;
                    simplified_approx_polygon.push_back(*it_end);
                    break;
                }
            }
        }
    }

    approxPoly.clear();
    copy(simplified_approx_polygon.begin(), simplified_approx_polygon.end(), back_inserter(approxPoly));
}

CNode* TargetPolygon::GetNeighbour_8(CNode* node, int i)
{
    //static int n_i[][8] ={{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1}};
    static int n_i[2][8] ={{0,1,1,1,0,-1,-1,-1}, {-1,-1,0,1,1,1,0,-1}};

    if(i >= 8) return NULL;
    //ROS_WARN("n_i : %u %d %d", i, n_i[0][i], n_i[1][i]);
    return GetNode(node->grd_x+n_i[0][i],node->grd_y+n_i[1][i]);
}

void TargetPolygon::ConvexHull()
{
    ch.clear();
    if(approxPoly.size()<3)
    {
        return;
    }

    // first node is the bottom most node
    unsigned first=0;
    for(unsigned int i=1; i<approxPoly.size(); i++)
        if(approxPoly[first]->pos[1]> approxPoly[i]->pos[1])
            first = i;

    ch.push_back(approxPoly[first]);
    //v.erase(v.begin()+first);

    center = approxPoly[0]->pos;

    if(approxPoly.size()<=1)
        return;

    //second node
    unsigned second=1;
    Vector<3> p1 = ch[0]->pos - makeVector(-1, 0, 0);
    Vector<3> p2 = ch.back()->pos;

    double ang = ANGLE(p1, p2, approxPoly[1]->pos);
    for(unsigned int i=2; i<approxPoly.size(); i++)
    {
        double a = ANGLE(p1, p2, approxPoly[i]->pos);
        if(a> ang)
        {
            ang = a;
            second = i;
        }
    }

    ch.push_back(approxPoly[second]);

    if(approxPoly.size() <=2)
        return;

    //check if the cells form a line a line
    isLine = true;
    Vector<3> p_0 = approxPoly[0]->pos;
    Vector<3> p_1 = approxPoly[1]->pos;

    for(size_t i=2; i < approxPoly.size() && isLine; i++)
    {
        Vector<3> p_n = approxPoly[i]->pos;
        if(fabs((p_0[1]-p_1[1])*(p_0[0]-p_n[0]) - (p_0[1]-p_n[1])*(p_0[0]-p_1[0])) > 0.01)
        {
            isLine = false;
        }
    }

    if(isLine)
    {
        ch.clear();
        std::copy(approxPoly.begin(), approxPoly.end(), std::back_inserter(ch));
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
            for(unsigned int i=0; i<approxPoly.size(); i++)
            {
                if(approxPoly[i] == ch.back())
                    continue;

                double a = ANGLE(p1, p2, approxPoly[i]->pos);
                if(a> ang)
                {
                    ang = a;
                    next = i;
                }
            }

            if(next > -1)
            {
                if(std::find(ch.begin(), ch.end(), approxPoly[next]) != ch.end())
                    break;
                else
                    ch.push_back(approxPoly[next]);
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

    std::reverse(ch.begin(), ch.end());
}

void TargetPolygon::FindBaseEdge()
{    
    base_idx[0] = -1;
    base_idx[1] = -1;

    if(ch.size()<3)
        return;

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

double TargetPolygon::pointToLineDist(Vector<3> p1, Vector<3> p2, Vector<3> x)
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

    if(ch.size() < 3)
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
//    if(ignored)
//        return;

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
//    glColor3f(1, 0.1, 0.1);
//    glLineWidth(15);
//    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//    //glBegin(GL_POLYGON);
//    glBegin(GL_POINTS);
//    for(unsigned int i=0; i<ch.size();i++)
//    {
//        glColor3f(((double)i)/ch.size(), 0,0);
//        TooN::Vector<3> p1 = ch[i]->GetMAVWaypoint();
//        glVertex3f(p1[0],p1[1],p1[2]);
//    }
//    glEnd();

    glColor3f(0, 0, 1);
    glLineWidth(5);
    glPolygonMode(GL_FRONT, GL_LINE);
    //glBegin(GL_POLYGON);
    glBegin(GL_LINES);
    for(size_t i=0; i<approxPoly.size(); i++)
    {
        glColor3f(0,0,1);
        TooN::Vector<3> p1 = approxPoly[i]->GetMAVWaypoint();
        TooN::Vector<3> p2 = approxPoly[(i+1)%approxPoly.size()]->GetMAVWaypoint();
        glVertex3f(p1[0],p1[1],p1[2]);
        glVertex3f(p2[0],p2[1],p2[2]);
    }
    glEnd();

    glColor3f(0, 0, 1);
    glPointSize(6);
    glBegin(GL_POINTS);
    for(unsigned int i=0; i<boundaryNodes.size();i++)
    {
        glColor3f(0,((double)i)/boundaryNodes.size(),((double)i)/boundaryNodes.size());
        TooN::Vector<3> p1 = boundaryNodes[i]->GetMAVWaypoint();
        glVertex3f(p1[0],p1[1],p1[2]);
    }
    glEnd();


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

//    for(int i=0; i <4; i++)
//    {
//        if(boundaryFLags[i])
//        {
//            double dx = (i==0)?-3:((i==1)?3:0);
//            double dy = (i==2)? 3:((i==3)?-3:0);

//            glLineWidth(5);
//            glBegin(GL_LINES);
//            glColor3f(1,1,0);
//            glVertex3f(center[0], center[1], center[2]);
//            glColor3f(0,1,1);
//            glVertex3f(center[0]+dx , center[1]+dy, center[2]);
//            glEnd();
//        }
//    }
}
