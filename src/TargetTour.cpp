#include "TargetTour.h"
#include <algorithm>

int TargetComp(TargetPolygon* tp1, TargetPolygon* tp2){return tp1->GetCenter()[0] > tp2->GetCenter()[0];}

int RandomNum (int i)
{
    return std::rand()%i;
}

double TargetTour::GetTargetTour(vector<TargetPolygon*> &targets, const Vector<3>& start, const Vector<3>& end)
{
    long unsigned int c = 0;
    vector<TargetPolygon*> tmp;
    copy(targets.begin(), targets.end(), back_inserter(tmp));

    double cost = 9999999999;

    std::sort(tmp.begin(), tmp.end(), TargetComp);

    if(targets.size() > 7)
    {
        for(unsigned int i=0; i<10000; i++)
        {
            std::random_shuffle(tmp.begin(), tmp.end(), RandomNum);
            double pcost = GetTourCost(tmp, start, end);
            if(pcost < cost)
            {
                cost = pcost;
                targets.clear();
                copy(tmp.begin(), tmp.end(), back_inserter(targets));
            }
        }
    }
    else
    {
        do
        {
            if(c++ > 1000)
            {
                c=0;
                ROS_INFO(".");
            }

            double pcost = GetTourCost(tmp, start, end);
            if(pcost < cost)
            {
                cost = pcost;
                targets.clear();
                copy(tmp.begin(), tmp.end(), back_inserter(targets));
            }

        }while(next_permutation(targets.begin(), targets.end()));
    }

    return cost;
}

double TargetTour::GetTargetTour_Utility(vector<TargetPolygon*> &targets, const Vector<3>& start, const Vector<3>& end)
{
    vector<TargetPolygon*> tmp;
    copy(targets.begin(), targets.end(), back_inserter(tmp));
    targets.clear();

    while(!tmp.empty())
    {
        auto x_cur = start;
        if(!targets.empty())
        {
            x_cur = targets.back()->LastLMPos();
        }

        auto next = tmp.begin();
        double b_c=9999;
        double b_v = 0;

        for(auto it=tmp.begin(); it != tmp.end(); it++)
        {
            double c = sqrt((x_cur-(*it)->FirstLMPos())*(x_cur-(*it)->FirstLMPos()));
            double v = (*it)->GetTargetRegionsArea();

            if(v/c > b_v/b_c)
            {
                next = it;
            }
        }

        targets.push_back(*next);
        tmp.erase(next);
    }

    return 0;
}

double TargetTour::GetTourCost(vector<TargetPolygon *> &targets, const Vector<3>& start, const Vector<3>& end)
{
    vector<Vector<3> > wps;
    wps.push_back(start);
    size_t size = targets.size();
    for(size_t i=0; i<size; ++i)
    {
        targets[i]->GetLawnmowerPlan(wps);
    }

    wps.push_back(end);

    return GetPlanExecutionTime(wps, start, end, true, true);
}


double TargetTour::GetPlanExecutionTime(std::vector<TooN::Vector<3> > & wps, const TooN::Vector<3>& curpos,
                                        const TooN::Vector<3>& endpos, bool initialTurn, bool endTurn)
{
    if(wps.empty())
        return 0;

    double t=0;
    double dist = 0;

    dist += sqrt(D2(curpos, wps.front()));
    dist += sqrt(D2(wps.back(), endpos));

    for(size_t i=0; i+1 < wps.size(); i++)
    {
        dist += sqrt(D2(wps[i], wps[i+1]));
    }

    t = dist/NUCParam::average_speed;

    if(wps.size() >=2)
    {
        t += COLLINEAR(curpos, wps[0], wps[1]) * NUCParam::turning_time;
        for(size_t i=1; i+1 < wps.size(); i++)
        {
            t += COLLINEAR(wps[i-1], wps[i], wps[i+1]) * NUCParam::turning_time;
        }

        t += COLLINEAR(wps[wps.size()-2], wps.back(), endpos) * NUCParam::turning_time;
    }

    if(wps.size() == 1)
    {
        t += COLLINEAR(curpos, wps[0], endpos) * NUCParam::turning_time;
    }

    if(initialTurn)
        t += NUCParam::turning_time;

    if(endTurn)
        t += NUCParam::turning_time;

    return t;
}

double TargetTour::GetPlanExecutionTime(std::vector<CNode*> & wps, const TooN::Vector<3>& curpos,
                                        const TooN::Vector<3>& endpos, bool initialTurn, bool endTurn)
{
    if(wps.empty())
    {
        return sqrt(D2(curpos, endpos))/NUCParam::average_speed + NUCParam::turning_time;
    }

    double t=0;
    double dist = 0;

    dist += sqrt(D2(wps.back()->GetMAVWaypoint(), endpos));

    for(size_t i=0; i+1 < wps.size(); i++)
    {
        dist += sqrt(D2(wps[i]->GetMAVWaypoint(), wps[i+1]->GetMAVWaypoint()));
    }

    t = dist/NUCParam::average_speed;

    if(wps.size() >=2)
    {
        for(size_t i=1; i+1 < wps.size(); i++)
        {
            t += COLLINEAR(wps[i-1]->GetMAVWaypoint(), wps[i]->GetMAVWaypoint(), wps[i+1]->GetMAVWaypoint()) * NUCParam::turning_time;
        }

        t += COLLINEAR(wps[wps.size()-1]->GetMAVWaypoint(), wps.back()->GetMAVWaypoint(), endpos) * NUCParam::turning_time;
    }

    if(wps.size() == 1)
    {
        t += /*COLLINEAR(curpos, wps[0]->GetMAVWaypoint(), endpos) */ NUCParam::turning_time;
    }

    if(initialTurn)
        t += NUCParam::turning_time;

    if(endTurn)
        t += NUCParam::turning_time;

    return t;
}

double TargetTour::GetPlanExecutionTime(vector<Vector<3> > &wps, bool ignoreFirstSegment, bool ignoreLastSegment)
{
    if(wps.empty())
        return 0;

    double t=0;
    double dist = 0;

    for(size_t i=0; i+1 < wps.size(); i++)
    {
        if(ignoreFirstSegment && i==0)
            continue;

        if(ignoreLastSegment && i+2==wps.size())
            continue;

        dist += sqrt(D2(wps[i], wps[i+1]));
    }

    t = dist/NUCParam::average_speed;

    if(wps.size() >=3)
    {
        for(size_t i=0; i+2 < wps.size(); i++)
        {
            t += COLLINEAR(wps[i], wps[i+1], wps[i+2]) * NUCParam::turning_time;
        }
    }

    return t;
}
