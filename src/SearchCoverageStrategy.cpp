#include "SearchCoverageStrategy.h"
#include "GL/glut.h"
#include "NUCParam.h"
#include <math.h>
#include <vector>
#include "TargetPolygon.h"
#include "TSP.h"
#include "TargetTour.h"

using namespace std;
using namespace TooN;

SearchCoverageStrategy::SearchCoverageStrategy(CNode *root)
{
    dummy = new CNode(makeVector(0,0,0,0));
    dummy->visited = false;
    dummy->depth = dummy->maxDepth;

    search_cell_size = 0;

    //cellW = 1;
    cluster_n=0;
    cutoff_prob = 0.60;
    remaining_time = NUCParam::time_limit;

    tree = root;
    tree->SetTreeVisited(false);
    SetupGrid(root);    
    GenerateLawnmower();
    startPos = makeVector(0,0,0);
    prevGoal = startPos;
}

SearchCoverageStrategy::~SearchCoverageStrategy()
{
    while(!nodeStack.empty())
    {
        CNode * n= nodeStack.back();
        nodeStack.pop_back();
        delete n;
    }

    while(!visitedNodes.empty())
    {
        CNode * n= visitedNodes.back();
        visitedNodes.pop_back();
        delete n;
    }

    delete dummy;
}

void SearchCoverageStrategy::GenerateLawnmower()
{
    int cell_in_lanes = ceil(((double)s)/NUCParam::lm_tracks);
    search_cell_size = cell_in_lanes;

    ROS_INFO("Track size: %d", cell_in_lanes);

    CNode* leaf = GetNode(0,0);
    double cellW = fabs(leaf->GetFootPrint()[0] - leaf->GetFootPrint()[2]);

    Rect r = tree->GetFootPrint();

    for(int i=0; i< NUCParam::lm_tracks; i++)
    {
        vector<CNode*> lanes;
        for(int j=0; j< NUCParam::lm_tracks; j++)
        {
            Rect nr;
            nr[0] = r[0] + i*cell_in_lanes*cellW;
            nr[1] = r[1] + j*cell_in_lanes*cellW;
            nr[2] = r[0] + (i+1)*cell_in_lanes*cellW;
            nr[3] = r[1] + (j+1)*cell_in_lanes*cellW;

            CNode * node = new CNode(nr, false);
            node->grd_x = i;
            node->grd_y = j;
            node->searchNode = true;
            lanes.push_back(node);
            FindSubCells(node);
        }

        copy(lanes.begin(), lanes.end(), back_inserter(search_grid));

        if(i%2)
            reverse(lanes.begin(), lanes.end());

        copy(lanes.begin(), lanes.end(), back_inserter(nodeStack));
    }

//    CNode * startNode = tree->GetNearestLeaf(makeVector(r[0],r[1],0), NUCParam::lm_height);
//    ROS_INFO("lm_height: %d depth: %d maxdepth: %d", NUCParam::lm_height, startNode->depth, CNode::maxDepth);
//    Vector<3> startPos = startNode->GetPos();

//    Rect rc = startNode->GetFootPrint();
//    double ld = sqrt((rc[0]-rc[2])*(rc[1]-rc[3]));
//    int n = l/ld;


//    printf("LM: n:%d l:%f ld:%f \n", n, l, ld);

//    for(int i=0; i< n; i++)
//        for(int j=0; j< n; j++)
//        {
//            int jj = (i%2 == 0)?j:n-j-1;
//            Vector<3> npos = startPos + makeVector(((double)i)*ld, ((double)jj)*ld, 0);
//            nodeStack.push_back(tree->GetNearestLeaf(npos,  NUCParam::lm_height));
//        }
}

CNode* SearchCoverageStrategy::GetNextNode()
{
    static bool returned_home = false;

    CNode * result = NULL;

    if(!target_lms.empty())
    {
        dummy->pos = target_lms.front();
        dummy->visited = false;
        target_lms.erase(target_lms.begin());
        result = dummy;
    }
    else if(!nodeStack.empty())
    {
        result = nodeStack.front();
        nodeStack.erase(nodeStack.begin());
    }
    else if(!returned_home)
    {
        returned_home = true;
        dummy->pos = startPos;
        dummy->visited = false;
        result = dummy;
    }

    UpdateRemainingTime(result);

    return result;
}

void SearchCoverageStrategy::UpdateRemainingTime(CNode *node)
{

    static bool first = true;
    static Vector<3> p1 = makeVector(0,0,0);
    static Vector<3> p2 = makeVector(0,0,0);

    if(!node)
        return;

    if(first)
    {
        first = false;
        remaining_time -= sqrt(D2(p1, node->GetMAVWaypoint()))/NUCParam::average_speed;
    }
    else
    {
        remaining_time -= COLLINEAR(p2, p1, node->GetMAVWaypoint()) * NUCParam::turning_time;
        remaining_time -= sqrt(D2(p1, node->GetMAVWaypoint()))/NUCParam::average_speed;
    }

    p2 = p1;
    p1 = node->GetMAVWaypoint();

    ROS_INFO("Remaining_time: %.2f", remaining_time);

}

void SearchCoverageStrategy::ReachedNode(CNode *node)
{
    bool reachedSearchNode = node->searchNode;//(node->depth == (node->maxDepth - NUCParam::lm_height));

    vector<TargetPolygon*> newTargets;

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

        //last_c = targets.size();
        FindClusters(true, newTargets);

        //ROS_INFO("#clusters: %u", targets.size());
    }

    if(NUCParam::policy == "greedy")
    {
        OnReachedNode_GreedyPolicy(node, newTargets, reachedSearchNode);
    }
    else if(NUCParam::policy == "delayed_greedy")
    {
        //ROS_INFO("Calling delayed_greedy .. ");
        OnReachedNode_DelayedGreedyPolicy(node, newTargets, reachedSearchNode);
        //ROS_INFO("Returning from delayed_greedy .. ");
    }
    else if(NUCParam::policy == "delayed")
    {
        OnReachedNode_DelayedPolicy(node, newTargets, reachedSearchNode);
    }

    prevGoal = node->GetMAVWaypoint();

    //ROS_INFO("returning from on_reached_goal ...");

}

void SearchCoverageStrategy::OnReachedNode_GreedyPolicy(CNode *node, vector<TargetPolygon*> &newTargets, bool searchNode)
{
    if(searchNode && !newTargets.empty())
    {

        //join the targets of the current cell
        //SimplifyTargetSet(newTargets);

//        if(targets.size()-newTargetIdxBegin >= 2)
//        {
//            TargetPolygon *t = targets[newTargetIdxBegin];
//            while(targets.size() > newTargetIdxBegin + 1)
//            {
//                TargetPolygon * tmp = targets.back();
//                targets.pop_back();
//               // ROS_INFO("here1 %u %u",newTargetIdxBegin, targets.size());
//                t->AddPolygon(tmp);
//               // ROS_INFO("here1 1.5 %u %u",newTargetIdxBegin, targets.size());

//                delete tmp;
//            }
//        }

        //            for(size_t i=0; i < newTargets.size(); i++)
        //            {
        //                newTargets[i]->GetLawnmowerPlan(target_lms);
        //                newTargets[i]->MarkAsVisited();
        //                targets.push_back(newTargets[i]);
        //            }

        //            vector<Vector<3> > tmp_plan;
        //            tmp_plan.push_back(prevGoal);
        //            tmp_plan.push_back(node->GetMAVWaypoint());
        //            copy(target_lms.begin(), target_lms.end(), back_inserter(tmp_plan));
        //            for(size_t i = 0; i < nodeStack.size(); i++)
        //                tmp_plan.push_back(nodeStack[i]->GetMAVWaypoint());
        //            tmp_plan.push_back(startPos);

        //            double total_time = GetPlanExecutionTime(tmp_plan, true, false);

        //            if(total_time > remaining_time)
        //            {
        //                if(targets.size() > newTargetIdxBegin)
        //                    targets[newTargetIdxBegin]->SetVisited(false);

        //                target_lms.clear();
        //            }
        //            else
        //            {
        //                if(targets.size() > newTargetIdxBegin)
        //                    targets[newTargetIdxBegin]->SetVisited(true);
        //            }


        long int selector = 1;
        long int maxSelector = 1 << newTargets.size();
        long int bestSelector = 0;
        double bestArea = 0;

        // keep track of the min cost plan
        // in case cannot make complete coverage
        double minCost = 99999999;
        long int minCostSelector = 0;

        bool flagAll = false;
        if(newTargets.size() > 63)
            flagAll = true;

        for(; selector < maxSelector && !flagAll; selector++)
        {
            double area=0;
            vector<CNode*> unionCells;
            for(size_t i=0; i < newTargets.size(); i++ )
            {
                long int b = 1 << (i);
                if(b & selector)
                {
                    newTargets[i]->GetCells(unionCells);
                    area += newTargets[i]->GetTargetRegionsArea();
                }
            }

            if(unionCells.empty())
                continue;

            TargetPolygon * tmpPoly = new TargetPolygon(unionCells, NULL);
            vector<Vector<3> > tmp_target_lm;
            tmpPoly->GetLawnmowerPlan(tmp_target_lm);

            // generate a plan which visits the current target and the rest of the search pattern
            vector<Vector<3> > tmp_plan;
            tmp_plan.push_back(prevGoal);
            tmp_plan.push_back(node->GetMAVWaypoint());
            copy(tmp_target_lm.begin(), tmp_target_lm.end(), back_inserter(tmp_plan));
            for(size_t i = 0; i < nodeStack.size(); i++)
                tmp_plan.push_back(nodeStack[i]->GetMAVWaypoint());
            tmp_plan.push_back(startPos);

            double total_time = TargetTour::GetPlanExecutionTime(tmp_plan, true, false);

            if(total_time < remaining_time)
            {
                if(bestArea < area)
                {
                    bestArea = area;
                    bestSelector = selector;
                }
            }

            if(total_time < minCost)
            {
                minCost = total_time;
                minCostSelector = selector;
            }
        }


        // if complete coverage was impossible
        // create partial coverage
        if(bestSelector == 0)
            bestSelector = minCostSelector;

        int firstTarget = -1;

        if(flagAll)
            firstTarget = 0;

        // Add the selected polygons to the first selected polygon
        for(size_t i=0; i < newTargets.size(); i++ )
        {
            long int b = 1 << (i);
            if(b & bestSelector || flagAll)
            {
                if(firstTarget > -1)
                {
                    newTargets[firstTarget]->AddPolygon(newTargets[i]);
                }
                else
                {
                    firstTarget = i;
                }
            }
        }


        if(firstTarget == -1)
        {
            ///for(size_t i=0; i < newTargets.size(); i++)
            //    targets.push_back(newTargets[i]);
            return;
        }

        // delete the selected polygons except the first one
        int sze = newTargets.size()-1;
        for(int i=sze; i > 0; i-- )
        {
            long int b = 1 << (i);
            if(b & bestSelector || flagAll)
            {
                TargetPolygon * tmp = newTargets[i];
                newTargets.erase(newTargets.begin()+i);
                delete tmp;
            }
        }

        // add the aggregate lawnmower plan to the current plan
        for(size_t i=0; i < newTargets.size(); i++)
        {
            if((int)i == firstTarget)
            {
                newTargets[i]->GetLawnmowerPlan(target_lms);
                newTargets[i]->MarkAsVisited();

                while(!target_lms.empty())
                {
                    //check if we should create partial plan
                    vector<Vector<3> > tmp_plan;
                    tmp_plan.push_back(prevGoal);
                    tmp_plan.push_back(node->GetMAVWaypoint());
                    copy(target_lms.begin(), target_lms.end(), back_inserter(tmp_plan));
                    for(size_t i = 0; i < nodeStack.size(); i++)
                        tmp_plan.push_back(nodeStack[i]->GetMAVWaypoint());
                    tmp_plan.push_back(startPos);

                    double total_time = TargetTour::GetPlanExecutionTime(tmp_plan, true, false);

                    if(total_time < remaining_time)
                    {
                        targets.push_back(newTargets[i]);
                        break;
                    }
                    else
                       target_lms.pop_back();
                }

            }
        }
    }
}

void SearchCoverageStrategy::OnReachedNode_DelayedPolicy(CNode *node, vector<TargetPolygon*> &newTargets, bool searchNode)
{
    if(searchNode)
    {
        if(nodeStack.empty())
        {
            CleanupTargets();
            FindClusters(false, targets);


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
            for(size_t i=0; i < tsp_target.size(); i++)
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
                else if(i+1 == tsp_target.size())
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

void SearchCoverageStrategy::OnReachedNode_DelayedGreedyPolicy(CNode *node, vector<TargetPolygon*> &newTargets, bool searchNode)
{

    if(searchNode)
    {
        for(size_t i=0; i < newTargets.size(); i++)
        {
           SetPolygonBoundaryFlags(newTargets[i], node);
        }


        for(size_t i=0; i < newTargets.size(); i++)
        {
            if(newTargets.size()==1)
            {
                gc.AddNode(newTargets[0]);
            }

            for(size_t j=i; j < newTargets.size(); j++)
            {
                if( (!newTargets[j]->GetBoundaryFlag(TargetPolygon::L) &&
                     !newTargets[j]->GetBoundaryFlag(TargetPolygon::R) &&
                     !newTargets[j]->GetBoundaryFlag(TargetPolygon::D) &&
                     !newTargets[j]->GetBoundaryFlag(TargetPolygon::U)) ||
                    (!newTargets[i]->GetBoundaryFlag(TargetPolygon::L) &&
                     !newTargets[i]->GetBoundaryFlag(TargetPolygon::R) &&
                     !newTargets[i]->GetBoundaryFlag(TargetPolygon::D) &&
                     !newTargets[i]->GetBoundaryFlag(TargetPolygon::U)))
                {
                    //ROS_INFO("Adding virtual edge...");
                    gc.AddEdge(newTargets[i], newTargets[j], true);
                }
            }

            if(newTargets[i]->GetBoundaryFlag(TargetPolygon::L))
            {
                int gx = node->grd_x-1;
                int gy = node->grd_y;
                CNode* ln = GetSearchNode(gx, gy);

                if(ln && ln->visited)
                {
                    for(size_t j = 0; j < ln->targets.size(); j++)
                    {
                        if(ln->targets[j]->GetBoundaryFlag(TargetPolygon::R))
                        {
                            gc.AddEdge(newTargets[i], ln->targets[j], false);
                        }
                    }
                }
            }

            if(newTargets[i]->GetBoundaryFlag(TargetPolygon::R))
            {
                int gx = node->grd_x+1;
                int gy = node->grd_y;
                CNode* ln = GetSearchNode(gx, gy);

                if(ln && ln->visited)
                {
                    for(size_t j = 0; j < ln->targets.size(); j++)
                    {
                        if(ln->targets[j]->GetBoundaryFlag(TargetPolygon::L))
                        {
                            gc.AddEdge(newTargets[i], ln->targets[j], false);
                        }
                    }
                }
            }

            if(newTargets[i]->GetBoundaryFlag(TargetPolygon::U))
            {
                int gx = node->grd_x;
                int gy = node->grd_y+1;
                CNode* ln = GetSearchNode(gx, gy);

                if(ln && ln->visited)
                {
                    for(size_t j = 0; j < ln->targets.size(); j++)
                    {
                        if(ln->targets[j]->GetBoundaryFlag(TargetPolygon::D))
                        {
                            gc.AddEdge(newTargets[i], ln->targets[j], false);
                        }
                    }
                }
            }

            if(newTargets[i]->GetBoundaryFlag(TargetPolygon::D))
            {
                int gx = node->grd_x;
                int gy = node->grd_y-1;
                CNode* ln = GetSearchNode(gx, gy);

                if(ln && ln->visited)
                {
                    for(size_t j = 0; j < ln->targets.size(); j++)
                    {
                        if(ln->targets[j]->GetBoundaryFlag(TargetPolygon::U))
                        {
                            gc.AddEdge(newTargets[i], ln->targets[j], false);
                        }
                    }
                }
            }
        }

        while(!components.empty())
        {
            vector<TargetPolygon*> * v = components.back();
            components.pop_back();
            v->clear();
            delete v;
        }

        gc.GetConnectedComponents(components);

        std::copy(newTargets.begin(), newTargets.end(), std::back_inserter(targets));

    }
}

void SearchCoverageStrategy::SetPolygonBoundaryFlags(TargetPolygon * plg, CNode* node)
{
    size_t distThresh = 3;

    for(TargetPolygon::SIDE side = (TargetPolygon::SIDE)0; side < TargetPolygon::ALL; side = (TargetPolygon::SIDE)((int)side+1))
    {
        if(side == TargetPolygon::R)
        {
            size_t minDist = 9999;
            vector<CNode*> side_space;
            vector<CNode*> target_cells;
            plg->GetCells(target_cells);
            for(size_t j=0; j<target_cells.size(); j++)
            {
                if(!InSearchGridBoundary(node->grd_x+1,node->grd_y))
                    break;

                int ii = (node->grd_x+1)*search_cell_size-1;
                int jj = target_cells[j]->grd_y;

                vector<CNode*> tmp;
                bool onTheBoundary = true;
                for(; ii > (node->grd_x)*search_cell_size-1; ii--)
                {
                    CNode * nc = GetNode(ii,jj);
                    if(nc->label == -1)
                    {
                        double d = (nc->colorBasis-makeVector(1,1,1))*(nc->colorBasis-makeVector(1,1,1));

                        if(d < 0.00001)
                        {
                            tmp.push_back(nc);
                        }
                        else
                        {
                            onTheBoundary = false;
                            break;
                        }

                        nc->colorBasis = makeVector(1,1,1);
                        nc->SetPrior(1);
                    }
                    else if(nc->label == target_cells[j]->label)
                    {
                        break;
                    }
                    else
                    {
                        while(!tmp.empty())
                        {
                            tmp.back()->colorBasis = makeVector(0,0,0);
                            tmp.pop_back();
                        }

                        onTheBoundary = false;
                        break;
                    }
                }

                if(onTheBoundary)
                {
                    if(minDist > tmp.size())
                        minDist = tmp.size();

                    std::copy(tmp.begin(), tmp.end(), std::back_inserter(side_space));
                }
            }

            if(minDist < distThresh)
            {
                plg->SetBoundaryFlag(side, 1);
                //newTargets[i]->SetPolygonColor(makeVector(1,0,0));
            }
            else
            {
                plg->SetBoundaryFlag(side, 0);
            }
        }
        else if(side == TargetPolygon::L)
        {
            size_t minDist = 9999;
            vector<CNode*> side_space;
            vector<CNode*> target_cells;
            plg->GetCells(target_cells);
            for(size_t j=0; j<target_cells.size(); j++)
            {
                if(!InSearchGridBoundary(node->grd_x-1,node->grd_y))
                    break;

                int ii = (node->grd_x)*search_cell_size;
                int jj = target_cells[j]->grd_y;

                vector<CNode*> tmp;
                bool onTheBoundary = true;
                for(; ii < (node->grd_x+1)*search_cell_size; ii++)
                {
                    CNode * nc = GetNode(ii,jj);
                    if(nc->label == -1)
                    {
                        double d = (nc->colorBasis-makeVector(1,1,1))*(nc->colorBasis-makeVector(1,1,1));

                        if(d < 0.00001)
                        {
                            tmp.push_back(nc);
                        }
                        else
                        {
                            onTheBoundary = false;
                            break;
                        }

                        nc->colorBasis = makeVector(1,1,1);
                        nc->SetPrior(1);
                    }
                    else if(nc->label == target_cells[j]->label)
                    {
                        break;
                    }
                    else
                    {
                        while(!tmp.empty())
                        {
                            tmp.back()->colorBasis = makeVector(0,0,0);
                            tmp.pop_back();
                        }

                        onTheBoundary = false;
                        break;
                    }
                }

                if(onTheBoundary)
                {
                    if(minDist > tmp.size())
                        minDist = tmp.size();

                    std::copy(tmp.begin(), tmp.end(), std::back_inserter(side_space));
                }
            }

            if(minDist < distThresh)
            {
                plg->SetBoundaryFlag(side, 1);
                //newTargets[i]->SetPolygonColor(makeVector(1,0,0));
            }
            else
            {
                plg->SetBoundaryFlag(side, 0);
            }
        }
        else if(side == TargetPolygon::U)
        {
            size_t minDist = 9999;
            vector<CNode*> side_space;
            vector<CNode*> target_cells;
            plg->GetCells(target_cells);
            for(size_t j=0; j<target_cells.size(); j++)
            {
                if(!InSearchGridBoundary(node->grd_x,node->grd_y+1))
                    break;

                int ii = target_cells[j]->grd_x;
                int jj = (node->grd_y+1)*search_cell_size-1;

                vector<CNode*> tmp;
                bool onTheBoundary = true;
                for(; jj > (node->grd_y)*search_cell_size-1; jj--)
                {
                    CNode * nc = GetNode(ii,jj);
                    if(nc->label == -1)
                    {
                        double d = (nc->colorBasis-makeVector(1,1,1))*(nc->colorBasis-makeVector(1,1,1));

                        if(d < 0.00001)
                        {
                            tmp.push_back(nc);
                        }
                        else
                        {
                            onTheBoundary = false;
                            break;
                        }

                        nc->colorBasis = makeVector(1,1,1);
                        nc->SetPrior(1);
                    }
                    else if(nc->label == target_cells[j]->label)
                    {
                        break;
                    }
                    else
                    {
                        while(!tmp.empty())
                        {
                            tmp.back()->colorBasis = makeVector(0,0,0);
                            tmp.pop_back();
                        }
                        onTheBoundary = false;
                        break;
                    }
                }

                if(onTheBoundary)
                {
                    if(minDist > tmp.size())
                        minDist = tmp.size();

                    std::copy(tmp.begin(), tmp.end(), std::back_inserter(side_space));
                }
            }

            if(minDist < distThresh)
            {
                plg->SetBoundaryFlag(side, 1);
                //newTargets[i]->SetPolygonColor(makeVector(1,0,0));
            }
            else
            {
                plg->SetBoundaryFlag(side, 0);
            }
        }
        else if(side == TargetPolygon::D)
        {
            size_t minDist = 9999;
            vector<CNode*> side_space;
            vector<CNode*> target_cells;
            plg->GetCells(target_cells);
            for(size_t j=0; j<target_cells.size(); j++)
            {
                if(!InSearchGridBoundary(node->grd_x,node->grd_y-1))
                    break;

                int ii = target_cells[j]->grd_x;
                int jj = (node->grd_y)*search_cell_size;

                vector<CNode*> tmp;
                bool onTheBoundary = true;
                for(; jj < (node->grd_y+1)*search_cell_size; jj++)
                {
                    CNode * nc = GetNode(ii,jj);
                    if(nc->label == -1)
                    {
                        double d = (nc->colorBasis-makeVector(1,1,1))*(nc->colorBasis-makeVector(1,1,1));

                        if(d < 0.00001)
                        {
                            tmp.push_back(nc);
                        }
                        else
                        {
                            onTheBoundary = false;
                            break;
                        }

                        nc->colorBasis = makeVector(1,1,1);
                        nc->SetPrior(1);
                    }
                    else if(nc->label == target_cells[j]->label)
                    {
                        break;
                    }
                    else
                    {
                        while(!tmp.empty())
                        {
                            tmp.back()->colorBasis = makeVector(0,0,0);
                            tmp.pop_back();
                        }
                        onTheBoundary = false;
                        break;
                    }
                }

                if(onTheBoundary)
                {
                    if(minDist > tmp.size())
                        minDist = tmp.size();

                    std::copy(tmp.begin(), tmp.end(), std::back_inserter(side_space));
                }
            }

            if(minDist < distThresh)
            {
                plg->SetBoundaryFlag(side, 1);
                //newTargets[i]->SetPolygonColor(makeVector(1,0,0));
            }
            else
            {
                plg->SetBoundaryFlag(side, 0);
            }
        }
    }
}


void SearchCoverageStrategy::glDraw()
{    
    for(size_t j=0; j<targets.size(); j++)
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

    if(nodeStack.size() > 2)
    {
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

    gc.glDraw();

    for(size_t i=0; i < components.size(); i++)
        for(size_t j=0; j < components[i]->size(); j++)
        {
            char c = i%8;
            glPointSize(20);
            glColor3f(c&1, c&2, c&4);
            glBegin(GL_POINTS);
            Vector<3> cn =components[i]->at(j)->GetCenter();
            glVertex3f(cn[0],cn[1],cn[2]+1);
            glEnd();
        }

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
    size_t n = i*s+j;
    if(n < grid.size())
        return grid[n];
    else
    {
        ROS_WARN("GetNode: out of boundary access ...");
        return NULL;
    }
}

bool SearchCoverageStrategy::InSearchGridBoundary(int i, int j)
{
    if(i < 0 || j < 0 || i >= NUCParam::lm_tracks || j >= NUCParam::lm_tracks)
    {
        return false;
    }

    return true;
}

CNode * SearchCoverageStrategy::GetSearchNode(int i, int j)
{
    if(i < 0 || j < 0 || i >= NUCParam::lm_tracks || j >= NUCParam::lm_tracks)
    {
        ROS_WARN("GetSearchNode: out of boundary access ...");

        return NULL;
    }

    size_t n = i*NUCParam::lm_tracks + j;
    if(n < search_grid.size())
        return search_grid[n];
    else
    {
        ROS_WARN("GetSearchNode: out of boundary access ...");
        return NULL;
    }
}

void SearchCoverageStrategy::hanldeKeyPressed(std::map<unsigned char, bool> &key, bool &updateKey)
{
//    bool flag = false;
//    if(key['\''])
//    {
//        cutoff_prob = (cutoff_prob<1.0)?cutoff_prob+0.05:cutoff_prob;
//        updateKey = true;
//        flag = true;
//    }
//    else if(key[';'])
//    {
//        cutoff_prob = (cutoff_prob>0.0)?cutoff_prob-0.05:cutoff_prob;
//        updateKey = true;
//        flag = true;
//    }
//    else if(key['l'])
//    {
//        updateKey = true;
//    }

//    if(flag)
//    {
//        for(unsigned int i=0; i<visitedNodes.size(); i++)
//            visitedNodes[i]->GenerateTargets(cutoff_prob);
//        FindClusters(true);
//    }


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

void SearchCoverageStrategy::FindSubCells(CNode *n)
{
    vector<CNode*> stack;
    Rect r = n->GetFootPrint();

    stack.push_back(tree);
    while(!stack.empty())
    {
        CNode * node = stack.back();
        stack.pop_back();

        if(node->IsLeaf())
        {
            Vector<3> p = node->GetMAVWaypoint();
            if(IN(p,r))
            {
                n->children.push_back(node);
                node->searchParentNode = n;
            }
        }
        else
        {
            copy(node->children.begin(), node->children.end(), back_inserter(stack));
        }

    }
}

void SearchCoverageStrategy::FindClusters(bool incremental, vector<TargetPolygon*> &newTargets)
{
    static int cn = newTargets.size()-1;
    if(!incremental)
        cn = -1;

    size_t old_cluster_n = cn+1;

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
            if((int)c.size() < cn+1)
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

    for(size_t i = old_cluster_n; i< (size_t)cluster_n; i++)
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
        newTargets.push_back(t);
    }
}
