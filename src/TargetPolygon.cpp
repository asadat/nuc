#include "TargetPolygon.h"
#include <algorithm>

#define ANGLE(a,b,c) (acos( ((a-b)*(c-b)) / (sqrt((a-b)*(a-b))*sqrt((c-b)*(c-b))) ))

using namespace std;

TargetPolygon::TargetPolygon(vector<CNode *> &cs)
{
    std::copy(cs.begin(), cs.end(), std::back_inserter(cells));
    label = cells[0]->label;
    height = 0;
    base_idx[0] = -1;
    base_idx[1] = -1;
}

TargetPolygon::~TargetPolygon()
{
}

void TargetPolygon::ConvexHull()
{
    // first node is the bottom most node
    unsigned first=0;
    for(unsigned int i=1; i<cells.size(); i++)
        if(cells[first]->pos[1]> cells[i]->pos[1])
            first = i;

    ch.push_back(cells[first]);
    //v.erase(v.begin()+first);

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

    //int n=0;
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

        if(std::find(ch.begin(), ch.end(), cells[next]) != ch.end())
            break;
        else
            ch.push_back(cells[next]);
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
