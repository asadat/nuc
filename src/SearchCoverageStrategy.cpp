#include "SearchCoverageStrategy.h"
#include "GL/glut.h"
#include "NUCParam.h"
#include <math.h>
#include <vector>
#include "TargetPolygon.h"
#include "TSP.h"
#include "TargetTour.h"
#include "CompoundTarget.h"

using namespace std;
using namespace TooN;

SearchCoverageStrategy::SearchCoverageStrategy(CNode *root)
{
    high_res_coverage=0;

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
    CleanupTargets();
    CleanupComponents();

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

    if(!result)
    {
        ROS_INFO("Area covered with high resolution: %f.2 m^2", high_res_coverage);
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
        double turned = COLLINEAR(p2, p1, node->GetMAVWaypoint());
        remaining_time -= turned*NUCParam::turning_time;
        if(turned > 0.1)
        {
            turningLocations.push_back(p1);
        }

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
        ROS_INFO("Calling delayed_greedy .. ");
        OnReachedNode_DelayedGreedyPolicy(node, newTargets, reachedSearchNode);
        ROS_INFO("Returning from delayed_greedy .. ");
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
                    newTargets[i]->GetCells(unionCells, NULL);
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
                high_res_coverage += newTargets[i]->GetTargetRegionsArea();

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

void SearchCoverageStrategy::PartiallyCoverTargets(vector<CompoundTarget*> &cts, const double budget, TooN::Vector<3> cur_pos, TooN::Vector<3> next_pos)
{

}

void SearchCoverageStrategy::AddTargetsToComponentGenerators(vector<TargetPolygon *> &newTargets, CNode* node)
{
    for(size_t i=0; i < newTargets.size(); i++)
    {
       SetPolygonBoundaryFlags(newTargets[i], node, false);
    }

    for(size_t i=0; i < newTargets.size(); i++)
    {
        if(newTargets.size()==1)
        {
            gc.AddNode(newTargets[0]);
        }

        if(newTargets[i]->IsNonBoundaryTarget())
        {
            double bminDist = 99999999;
            TargetPolygon* nearestBoundaryTarget = NULL;

            for(size_t j=0; j<newTargets.size(); j++)
            {
                if(i==j)
                    continue;

                if(!newTargets[j]->IsNonBoundaryTarget())
                {
                    Vector<3> v = newTargets[i]->GetCenter()-newTargets[j]->GetCenter();
                    double dist = v*v;
                    if(bminDist > dist)
                    {
                        bminDist = dist;
                        nearestBoundaryTarget = newTargets[j];
                    }
                }
            }

            if(nearestBoundaryTarget != NULL)
            {
                gc.AddEdge(newTargets[i], nearestBoundaryTarget, true);
            }
            else
            {
                for(size_t j=0; j<newTargets.size(); j++)
                {
                    if(newTargets[j]->IsNonBoundaryTarget())
                        gc.AddEdge(newTargets[i], newTargets[j], true);
                }
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
                    if(!ln->targets[j]->ignored && ln->targets[j]->GetBoundaryFlag(TargetPolygon::R))
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
                    if(!ln->targets[j]->ignored && ln->targets[j]->GetBoundaryFlag(TargetPolygon::L))
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
                    if(!ln->targets[j]->ignored && ln->targets[j]->GetBoundaryFlag(TargetPolygon::D))
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
                    if(!ln->targets[j]->ignored && ln->targets[j]->GetBoundaryFlag(TargetPolygon::U))
                    {
                        gc.AddEdge(newTargets[i], ln->targets[j], false);
                    }
                }
            }
        }
    }
}

void SearchCoverageStrategy::OnReachedNode_DelayedGreedyPolicy(CNode *node, vector<TargetPolygon*> &newTargets, bool searchNode)
{
    if(searchNode)
    {
        for(size_t i=0; i<targets.size(); i++)
            targets[i]->UpdateIsVisited();

        AddTargetsToComponentGenerators(newTargets, node);

        targets2visit.clear();
        CleanupComponents();

        // get the targets
        gc.GetIntegratedComponents(integrated_components);

        for(size_t i=0; i<integrated_components.size(); i++)
        {
            SetCompoundTargetBoundaryFlags(integrated_components[i]);
            integrated_components[i]->SetIsCurChildFlag(node);
        }

        double coverage_time = TargetTour::GetPlanExecutionTime(nodeStack, node->GetMAVWaypoint(), startPos, true, false);
        double time_budget = remaining_time - coverage_time;

        vector<CompoundTarget*> cur_compound_targets;
        vector<CompoundTarget*> extensible_compound_targets;

        SeparateCompoundTargets(integrated_components, node, cur_compound_targets, extensible_compound_targets);
        bool case_1 = true;
        for(size_t i=0; i<extensible_compound_targets.size() && case_1; i++)
            if(extensible_compound_targets[i]->cur_child)
                case_1 = false;

        /* case (1): No extensible compound on the current search node
         *         => Calculate costs, solve knapsack, mark unselected
         *            non-extensible targets as ignored, if cur_targets
         *            are selected cover them and then continue the high
         *            level Lawnmower.
         */

        if(case_1)
        {
            double extra_time = 0;
            double knapsack_cost = 0;

            ROS_INFO("****** Case 1");
            SetupCostsValues_NoExtensibleTarget(cur_compound_targets, extensible_compound_targets, node);

            Knapsack<CompoundTarget> ns;
            for(size_t i=0; i<integrated_components.size(); i++)
                ns.AddItem(integrated_components[i],integrated_components[i]->value, integrated_components[i]->cost);

            double tmp_d = 0;
            ns.Solve(time_budget, targets2visit, tmp_d, knapsack_cost);
            extra_time = time_budget - knapsack_cost;

            ROS_INFO("Knapsack solution size: %lu", targets2visit.size());

            for(size_t i=0; i<cur_compound_targets.size(); i++)
            {
                cur_compound_targets[i]->SetVisited();

                if(targets2visit.find(cur_compound_targets[i]) != targets2visit.end())
                {
                    EnqueueCompoundTarget(cur_compound_targets[i]);
                    cur_compound_targets.erase(cur_compound_targets.begin()+(i++));
                }
            }

            PartiallyCoverTargets(cur_compound_targets, extra_time, node->GetMAVWaypoint(), node->GetMAVWaypoint());


        }
        else
            /* case (2): there is an extensible compound on the current search node
             *         => consider 2 options and select the one that has more reward
             *              option 1: delay current target
             *              option 2: do not delay current target
             *              In either case, compute the costs, solve the knapsack
             *              mark unselected non-extensible targets as ignored, if
             *              option 2 is selected and cur_target is among the knapsack solution
             *              cover it right away.
             */

        {
            ROS_INFO("****** Case 2");
            double delay_value = 0;
            double cover_value = 0;
            double delay_cost = 0;
            double cover_cost = 0;
            double extra_time_delay = 0;
            double extra_time_cover = 0;

            SetupCostsValues_WithExtensibleTarget(cur_compound_targets, extensible_compound_targets, node, true);
            ROS_INFO("#Integrated: %lu #cur: %lu #ext: %lu", integrated_components.size(), cur_compound_targets.size(), extensible_compound_targets.size());

            Knapsack<CompoundTarget> ns1;
            for(size_t i=0; i<integrated_components.size(); i++)
                ns1.AddItem(integrated_components[i],integrated_components[i]->value, integrated_components[i]->cost);

            set<CompoundTarget*> targets2visit1;
            ns1.Solve(time_budget, targets2visit1, delay_value, delay_cost);
            extra_time_delay = time_budget - delay_cost;

            ROS_INFO("Knapsack solution size when delaying: %lu budget:%f", targets2visit1.size(), time_budget);

            SetupCostsValues_WithExtensibleTarget(cur_compound_targets, extensible_compound_targets, node, false);

            Knapsack<CompoundTarget> ns2;
            for(size_t i=0; i<integrated_components.size(); i++)
                ns2.AddItem(integrated_components[i],integrated_components[i]->value, integrated_components[i]->cost);

            set<CompoundTarget*> targets2visit2;
            ns2.Solve(time_budget, targets2visit2, cover_value, cover_cost);
            extra_time_cover = time_budget - cover_cost;
            ROS_INFO("Knapsack solution size when covering: %lu budget:%f", targets2visit2.size(),time_budget);

            ROS_INFO("Case2 ->   Value_delay: %f Value_cover: %f", delay_value, cover_value);

            bool delay_extensibles = false;

            if(delay_value < cover_value)
            {
                copy(targets2visit2.begin(), targets2visit2.end(), inserter(targets2visit, targets2visit.begin()));
                delay_extensibles = false;
            }
            else
            {
                copy(targets2visit1.begin(), targets2visit1.end(), inserter(targets2visit, targets2visit.begin()));
                delay_extensibles = true;
            }

            vector<CompoundTarget*> cur_unselected_compounds;

            for(size_t i=0; i<cur_compound_targets.size(); i++)
            {
                cur_compound_targets[i]->SetVisited();

                if(targets2visit.find(cur_compound_targets[i]) != targets2visit.end())
                {
                    EnqueueCompoundTarget(cur_compound_targets[i]);
                }
                else
                {
                    cur_unselected_compounds.push_back(cur_compound_targets[i]);
                }
            }

            if(!delay_extensibles)
            {
                for(size_t i=0; i<extensible_compound_targets.size(); i++)
                {
                    if(targets2visit.find(extensible_compound_targets[i]) != targets2visit.end())
                    {
                        extensible_compound_targets[i]->SetVisited();
                        EnqueueCompoundTarget(extensible_compound_targets[i]);
                    }
                    else if(extensible_compound_targets[i]->cur_child)
                    {
                        cur_unselected_compounds.push_back(extensible_compound_targets[i]);
                    }
                }
            }
            else
            {
                for(size_t i=0; i<extensible_compound_targets.size(); i++)
                {
                    if(targets2visit.find(extensible_compound_targets[i]) == targets2visit.end() && extensible_compound_targets[i]->cur_child)
                    {
                        cur_unselected_compounds.push_back(extensible_compound_targets[i]);
                    }
                }
            }

            PartiallyCoverTargets(cur_unselected_compounds, delay_extensibles?extra_time_delay:extra_time_cover, node->GetMAVWaypoint(), node->GetMAVWaypoint());

        }

        std::copy(newTargets.begin(), newTargets.end(), std::back_inserter(targets));
    }
}

void SearchCoverageStrategy::EnqueueCompoundTarget(CompoundTarget *ct)
{
    high_res_coverage += ct->value;
    ct->GetLawnmowerPlan(target_lms);
}

void SearchCoverageStrategy::SetupCostsValues_NoExtensibleTarget(std::vector<CompoundTarget *> &cur_targets,
                                              std::vector<CompoundTarget *> &extensible_targets, CNode* cur_node)
{
    TargetTour tt;
    vector<CNode*> nds;

    /* We will consider covering the current target from the current node
     * as well as from the unvisited neighbours. For the rest of the targets
     * it only suffices to consider unvisited neighbours.
     */

    for(size_t i=0; i < extensible_targets.size(); i++)
    {
        CompoundTarget &v = *extensible_targets[i];

        nds.clear();
        v.GetBoundarySeachNodes(nds);
        v.CalculateValue();

        double minCost = 99999999;
        CNode* minNode = NULL;
        for(size_t k=0; k<nds.size(); k++)
        {
            double c = tt.GetTargetTour(v.targets, nds[k]->GetMAVWaypoint(), nds[k]->GetMAVWaypoint());
            if(minCost > c)
            {
                minCost = c;
                minNode = nds[k];
            }
        }

        v.startNode = minNode;
        v.cost = minCost;
    }

    for(size_t i=0; i < cur_targets.size(); i++)
    {
        CompoundTarget &v = *cur_targets[i];

        v.CalculateValue();
        v.startNode = cur_node;
        v.cost = tt.GetTargetTour(v.targets, cur_node->GetMAVWaypoint(), cur_node->GetMAVWaypoint());
    }
}

void SearchCoverageStrategy::SetupCostsValues_WithExtensibleTarget(std::vector<CompoundTarget *> &cur_targets,
                                              std::vector<CompoundTarget *> &extensible_targets, CNode* cur_node, bool delay)
{
    TargetTour tt;
    vector<CNode*> nds;

    for(size_t i=0; i < extensible_targets.size(); i++)
    {
        CompoundTarget &v = *extensible_targets[i];

        nds.clear();
        v.GetBoundarySeachNodes(nds);
        v.CalculateValue();

        double minCost = 99999999;
        CNode* minNode = NULL;

        if(!v.cur_child || delay)
        {
            for(size_t k=0; k<nds.size(); k++)
            {
                double c = tt.GetTargetTour(v.targets, nds[k]->GetMAVWaypoint(), nds[k]->GetMAVWaypoint());
                if(minCost > c)
                {
                    minCost = c;
                    minNode = nds[k];
                }
            }
        }
        else
        {
            minCost = tt.GetTargetTour(v.targets, cur_node->GetMAVWaypoint(), cur_node->GetMAVWaypoint());
            minNode = cur_node;
        }

        ROS_INFO("Setting cost: %f cur_child:%d delay:%d #nds:%lu", minCost, v.cur_child, delay, nds.size());
        v.startNode = minNode;
        v.cost = minCost;
    }

    for(size_t i=0; i < cur_targets.size(); i++)
    {
        CompoundTarget &v = *cur_targets[i];

        v.CalculateValue();
        v.startNode = cur_node;
        v.cost = tt.GetTargetTour(v.targets, cur_node->GetMAVWaypoint(), cur_node->GetMAVWaypoint());
    }
}

void SearchCoverageStrategy::GetNearestStartCellAndCost(std::vector<CompoundTarget *> &cmpn, CNode* cur_node)
{
    TargetTour tt;
    vector<CNode*> nds;

    /* We will consider covering the current target from the current node
     * as well as from the unvisited neighbours. For the rest of the targets
     * it only suffices to consider unvisited neighbours.
     */

    copy(nodeStack.begin(), nodeStack.end(), back_inserter(nds));
    nds.insert(nds.begin(), cur_node);

    for(size_t i=0; i < cmpn.size(); i++)
    {
        double val = 0;
        vector<CNode*> parents;
        CompoundTarget &v = *cmpn[i];
        for(size_t j=0; j<v.targets.size(); j++)
        {
            val += v.targets[j]->GetTargetRegionsArea();
            copy(v.targets[j]->parentSearchNodes.begin(),v.targets[j]->parentSearchNodes.end(), back_inserter(parents));
        }

        // check if the cur_node is a parent of a sub-patch of the compound target
        // if yes, consider it as a waypoint to start coverage of the compound;
        bool consider_cur_node = false;
        for(size_t w=0; w<parents.size(); w++)
        {
            if(parents[w] == cur_node)
            {
                consider_cur_node = true;
                break;
            }
        }

        double minCost = 99999999;
        CNode* minNode = NULL;
        for(size_t k=0; k<parents.size(); k++)
        {
            for(size_t j=(consider_cur_node?0:1); j<nds.size(); j++)
            {
                if(!NeighboursNode(nds[j],parents[k]))
                    continue;

                double c = tt.GetTargetTour(v.targets, nds[j]->GetMAVWaypoint(), nds[j]->GetMAVWaypoint());
                if(minCost > c)
                {
                    minCost = c;
                    minNode = nds[j];
                }
            }
        }

        cmpn[i]->startNode = minNode;
        cmpn[i]->cost = minCost;
        cmpn[i]->value = val;

    }
}

void SearchCoverageStrategy::SeparateCompoundTargets(vector<CompoundTarget*> &all_targets, CNode* cur_search_node,
                             vector<CompoundTarget*> &cur_targets, vector<CompoundTarget*> &extensible_targets)
{
    for(size_t i=0; i<all_targets.size(); i++)
    {
        if(!all_targets[i]->IsExtensible())
            cur_targets.push_back(all_targets[i]);
        else
            extensible_targets.push_back(all_targets[i]);
    }
}

bool SearchCoverageStrategy::NeighboursNode(CNode *n1, CNode *n2)
{
    if(abs(n1->grd_x-n2->grd_x)+abs(n1->grd_y-n2->grd_y) <= 1)
        return true;

    return false;
}

void SearchCoverageStrategy::SetCompoundTargetBoundaryFlags(CompoundTarget *ct)
{
    ct->ResetBoundaries();

    for(size_t i=0; i<ct->targets.size(); i++)
    {
        for(set<CNode*>::iterator it=ct->targets[i]->parentSearchNodes.begin();
            it !=ct->targets[i]->parentSearchNodes.end(); it++)
        {
            this->SetPolygonBoundaryFlags(ct->targets[i], *it, true, ct->boundaryNodes);
        }
    }
}

void SearchCoverageStrategy::SetPolygonBoundaryFlags(TargetPolygon * plg, CNode* node, bool unvisitedBoundary, vector<CNode*> * bnodes)
{
    size_t distThresh = 3;

    vector<CNode*> target_cells;
    plg->GetCells(target_cells, node);

    for(TargetPolygon::SIDE side = (TargetPolygon::SIDE)0; side < TargetPolygon::ALL; side = (TargetPolygon::SIDE)((int)side+1))
    {
        if(side == TargetPolygon::R)
        {
            size_t minDist = 9999;
            vector<CNode*> side_space;

            if(!InSearchGridBoundary(node->grd_x+1,node->grd_y))
                continue;

            if(unvisitedBoundary && GetSearchNode(node->grd_x+1,node->grd_y)->visited)
                continue;

            for(size_t j=0; j<target_cells.size(); j++)
            {

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
                plg->OrBoundaryFlag(side, 1);
                if(bnodes)
                    bnodes[side].push_back(GetSearchNode(node->grd_x+1,node->grd_y));
                //newTargets[i]->SetPolygonColor(makeVector(1,0,0));
            }
            else
            {
                plg->OrBoundaryFlag(side, 0);
            }
        }
        else if(side == TargetPolygon::L)
        {
            size_t minDist = 9999;
            vector<CNode*> side_space;
            //vector<CNode*> target_cells;
            //plg->GetCells(target_cells, node);
            if(!InSearchGridBoundary(node->grd_x-1,node->grd_y))
                continue;

            if(unvisitedBoundary && GetSearchNode(node->grd_x-1,node->grd_y)->visited)
                continue;

            for(size_t j=0; j<target_cells.size(); j++)
            {
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
                plg->OrBoundaryFlag(side, 1);
                if(bnodes)
                    bnodes[side].push_back(GetSearchNode(node->grd_x-1,node->grd_y));

                //newTargets[i]->SetPolygonColor(makeVector(1,0,0));
            }
            else
            {
                plg->OrBoundaryFlag(side, 0);
            }
        }
        else if(side == TargetPolygon::U)
        {
            size_t minDist = 9999;
            vector<CNode*> side_space;

            if(!InSearchGridBoundary(node->grd_x,node->grd_y+1))
                continue;

            if(unvisitedBoundary && GetSearchNode(node->grd_x,node->grd_y+1)->visited)
                continue;

            for(size_t j=0; j<target_cells.size(); j++)
            {
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
                plg->OrBoundaryFlag(side, 1);
                if(bnodes)
                    bnodes[side].push_back(GetSearchNode(node->grd_x,node->grd_y+1));

                //newTargets[i]->SetPolygonColor(makeVector(1,0,0));
            }
            else
            {
                plg->OrBoundaryFlag(side, 0);
            }
        }
        else if(side == TargetPolygon::D)
        {
            size_t minDist = 9999;
            vector<CNode*> side_space;
            //vector<CNode*> target_cells;
            //plg->GetCells(target_cells, node);
            if(!InSearchGridBoundary(node->grd_x,node->grd_y-1))
                continue;

            if(unvisitedBoundary && GetSearchNode(node->grd_x,node->grd_y-1)->visited)
                continue;

            for(size_t j=0; j<target_cells.size(); j++)
            {
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
                plg->OrBoundaryFlag(side, 1);
                if(bnodes)
                    bnodes[side].push_back(GetSearchNode(node->grd_x,node->grd_y-1));

                //newTargets[i]->SetPolygonColor(makeVector(1,0,0));
            }
            else
            {
                plg->OrBoundaryFlag(side, 0);
            }
        }
    }
}


void SearchCoverageStrategy::glDraw()
{    
//    for(size_t j=0; j<targets.size(); j++)
//        targets[j]->glDraw();


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

   // gc.glDraw();

//    for(size_t i=0; i < components.size(); i++)
//        for(size_t j=0; j < components[i]->size(); j++)
//        {
//            char c = i%8;
//            glPointSize(20);
//            glColor3f(c&1, c&2, c&4);
//            glBegin(GL_POINTS);
//            Vector<3> cn =components[i]->at(j)->GetCenter();
//            glVertex3f(cn[0],cn[1],cn[2]+1);
//            glEnd();
//        }

    for(size_t i=0; i < integrated_components.size(); i++)
        integrated_components[i]->glDraw();

//    glPointSize(20);
//    glBegin(GL_POINTS);
//    for(size_t i =0; i<start_nodes.size(); i++)
//    {
//        char c = i%8;
//        glColor3f(c&1, c&2, c&4);
//        Vector<3> cn = start_nodes[i]->GetPos();
//        glVertex3f(cn[0],cn[1],cn[2]);
//    }
//    glEnd();

//    for(set<CompoundTarget*>::iterator it = targets2visit.begin(); it!=targets2visit.end(); it++)
//    {
//        (*it)->glDraw();
//        //if(start_nodes)
//    }

    glPointSize(5);
    glColor3f(1,1,0);
    glBegin(GL_POINTS);
    for(size_t i =0; i<turningLocations.size(); i++)
    {
        Vector<3> cn = turningLocations[i];
        glVertex3f(cn[0],cn[1],cn[2]);
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

void SearchCoverageStrategy::CleanupComponents()
{
    while(!components.empty())
    {
        CompoundTarget * v = components.back();
        components.pop_back();
        delete v;
    }

    while(!integrated_components.empty())
    {
        CompoundTarget * v = integrated_components.back();
        integrated_components.pop_back();

        while(!v->targets.empty())
        {
            TargetPolygon * t = v->targets.back();
            v->targets.pop_back();
            delete t;
        }

        delete v;
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
