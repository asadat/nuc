#include "SearchCoverageStrategy.h"
#include "GL/glut.h"
#include "NUCParam.h"
#include <math.h>
#include <vector>
#include "TargetPolygon.h"
#include "TSP.h"
#include "TargetTour.h"
#include "CompoundTarget.h"
#include "ScalarField.h"
#include <random>
#include <functional>

using namespace std;
using namespace TooN;

extern void glVertex(Vector<3>);

SearchCoverageStrategy::SearchCoverageStrategy(CNode *root)
{

    drawFootprints = false;
    high_res_coverage=0;
    high_res_coverage_true=0;

    dummy = new CNode(makeVector(0,0,0,0));
    dummy->visited = false;
    dummy->depth = dummy->maxDepth;

    search_cell_size = 0;

    //cellW = 1;
    cluster_n=0;
    cutoff_prob = 0.30;
    remaining_time = NUCParam::time_limit;

    tree = root;
    tree->SetTreeVisited(false);
    SetupGrid(root);    
    GenerateLawnmower();
    startPos = makeVector(0,0,0);
    prevGoal = startPos;

    GenerateEnvironment();
}

SearchCoverageStrategy::~SearchCoverageStrategy()
{
    drawFootprints = false;
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

void SearchCoverageStrategy::CellularAutomataStep(vector<CNode*>&regions)
{
    vector<CNode*> newregions;

    for(auto c: regions)
    {
        for(char selector = 1; selector <= 8; selector = selector<<1)
        {
            auto nb = c->GetNeighbourLeaf(selector&1,selector&2,selector&4,selector&8);
            if(nb)
            {
                if(nb->prior_cell < 0.4 && nb->imgPrior < 0.4)
                {
                    if(RAND(0,100) < 50)
                    {
                        nb->imgPrior = 0.5;
                        newregions.push_back(nb);
                    }
                }
            }
         }
    }

    regions.clear();
    for(auto c: newregions)
    {
        c->prior_cell = c->imgPrior;
        regions.push_back(c);
    }
}

void SearchCoverageStrategy::GenerateEnvironment(bool randomize)
{
    if(NUCParam::random_seed > 0 && !randomize)
    {
        used_seed = NUCParam::random_seed;
    }
    else
    {
        used_seed = time(NULL);
        ROS_INFO("Randomization Seed: %ld", used_seed);
    }

    srand(used_seed);


    for(auto &c: grid)
    {
        c->SetPrior(0.5);
        c->prior_cell = 0;
        c->imgPrior = 0.0;
    }

    int cluster_count = 4;
    vector<CNode*> growing_regions;
    for(int i=0; i<cluster_count; i++)
    {
       int gx = RAND(0, s);
       int gy = RAND(0, s);

       auto c = GetNode(gx, gy);
       if(c)
       {
            c->prior_cell = 0.5;
            c->imgPrior = 0.5;
            growing_regions.push_back(c);
       }

    }

    int iterations_count = ComputeCellularAutomataSteps(s*s*NUCParam::percent_interesting/100.0, cluster_count);
    for(int i=0; i<4*iterations_count; i++)
    {
        //ROS_INFO("iteration: %d", i);
        CellularAutomataStep(growing_regions);
    }


    int d=5;
    for(int i=0; i<s; i++)
    {
        for(int j=0; j<s; j++)
        {
            double p=0;
            int n=0;
            for(int ii=i-d; ii <= i+d; ii++)
                for(int jj=j-d; jj <= j+d; jj++)
                {
                    if(ii>=0 && ii<s && jj>=0 && jj<s)
                    {
                        n++;
                        p += GetNode(ii,jj)->prior_cell;
                    }
                }

            GetNode(i,j)->imgPrior = p/n;
        }
    }

}

int SearchCoverageStrategy::ComputeCellularAutomataSteps(int fib, int clusters_count)
{
    std::function<int(int)> fidx = [](int sum)->int
    {
        int s=1;
        for(int i=1; ;i++)
        {
            s += i*4;
            if(s > sum)
                return i;
        }
    };

    return fidx(fib/clusters_count);
}

Vector<3> SearchCoverageStrategy::GetNextSearchPos()
{
    if(SearchNodeExists())
        return nodeStack.front()->GetMAVWaypoint();
    else
        return startPos;
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
        Vector<3> tmp = dummy->pos;
        hi_res_waypoints.push_back(tmp);
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
        ROS_INFO("Area covered with high resolution: (%.2f,%.2f) percent: %0.2f policy: %s time_left: %0.2f seed: %ld", high_res_coverage, high_res_coverage_true, NUCParam::percent_interesting,
                 NUCParam::policy.c_str(), remaining_time, used_seed);
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

void SearchCoverageStrategy::SensingUpdate(TooN::Vector<3> pos)
{
    std::random_device rd;
    std::mt19937 e2(rd());
    ROS_INFO("Start Observing ...");
    Rect fp = GetCourseSurveyFootprint();
    TooN::Vector<2> fc = makeVector(0.5*(fp[0]+fp[2]), 0.5*(fp[1]+fp[3]));
    fp[0] -= fc[0] - pos[0];
    fp[1] -= fc[1] - pos[1];
    fp[2] -= fc[0] - pos[0];
    fp[3] -= fc[1] - pos[1];

    const int n = 2;
    double dx = fabs(fp[0]-fp[2])/n;
    double dy = fabs(fp[1]-fp[3])/n;
    //for(int k=0; k<5; k++)
    for(int i=0; i<n; i++)
        for(int j=0; j<n; j++)
        {
            TooN::Vector<3> p= makeVector(fp[0]+dx*(0.5+i), fp[1]+dy*(0.5+j), 0.0);
            auto leaf = tree->GetNearestLeaf(p);
            double x[]={leaf->GetMAVWaypoint()[0],leaf->GetMAVWaypoint()[1]};
            std::normal_distribution<> dist(leaf->imgPrior, NUCParam::gp_sigma);

            ScalarField::GetInstance()->add_pattern(x, dist(e2));
        }

    ROS_INFO("End Observing ...");
    ROS_INFO("Update GP ...");
    tree->UpdateGPValues();
    //node->GetNeighbourLeaf()
    ROS_INFO("Updated GP ...");
}

void SearchCoverageStrategy::ReachedNode(CNode *node)
{
    bool reachedSearchNode = node->searchNode;

    vector<TargetPolygon*> newTargets;

    if(reachedSearchNode)
    {
//        std::random_device rd;
//        std::mt19937 e2(rd());
//        ROS_INFO("Start Observing ...");
//        Rect fp = node->GetFootPrint();
//        const int n = 2;
//        double dx = fabs(fp[0]-fp[2])/n;
//        double dy = fabs(fp[1]-fp[3])/n;
//        //for(int k=0; k<5; k++)
//        for(int i=0; i<n; i++)
//            for(int j=0; j<n; j++)
//            {
//                TooN::Vector<3> p= makeVector(fp[0]+dx*(0.5+i), fp[1]+dy*(0.5+j), 0.0);
//                auto leaf = node->GetNearestLeaf(p);
//                double x[]={leaf->GetMAVWaypoint()[0],leaf->GetMAVWaypoint()[1]};
//                std::normal_distribution<> dist(leaf->imgPrior, NUCParam::gp_sigma);

//                ScalarField::GetInstance()->add_pattern(x, dist(e2));
//            }

//        ROS_INFO("End Observing ...");
//        ROS_INFO("Update GP ...");
//        tree->UpdateGPValues();
//        //node->GetNeighbourLeaf()
//        ROS_INFO("Updated GP ...");

        SensingUpdate(node->GetMAVWaypoint());

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


        //ROS_INFO("#clusters: %u", targets.size());
    }

    if(NUCParam::policy == "greedy")
    {
        ROS_INFO("start find cluster .. ");
        FindClusters(true, newTargets, node);
        ROS_INFO("end find cluster ... greedy policy ");
        OnReachedNode_GreedyPolicy(node, newTargets, reachedSearchNode);
        ROS_INFO("end greedy policy .. ");
    }
    else if(NUCParam::policy == "delayed_greedy")
    {
        ROS_INFO("start find cluster .. ");
        FindClusters(true, newTargets, node);
        ROS_INFO("end find cluster .. ");

        ROS_INFO("Calling delayed_greedy .. ");
        OnReachedNode_DelayedGreedyPolicy(node, newTargets, reachedSearchNode);
        ROS_INFO("Returning from delayed_greedy .. ");
    }
    else if(NUCParam::policy == "delayed")
    {
        FindClusters(true, newTargets);
        OnReachedNode_DelayedPolicy(node, newTargets, reachedSearchNode);
    }

    prevGoal = node->GetMAVWaypoint();

    //ROS_INFO("returning from on_reached_goal ...");
}

void SearchCoverageStrategy::OnReachedNode_GreedyPolicy(CNode *node, vector<TargetPolygon*> &newTargets, bool searchNode)
{
    if(searchNode && !newTargets.empty())
    {        
        CompoundTarget ct;
        for(size_t i=0; i<newTargets.size(); i++)
        {
            ROS_INFO("Target size: %lu lm: %lu", newTargets[i]->ch.size(), newTargets[i]->lm.size());
            if(newTargets[i]->HasParent(node))
                ct.AddTarget(newTargets[i]);
        }
        ROS_INFO("G1 ");
        TargetTour::GetTargetTour_Utility(ct.targets, node->GetMAVWaypoint(), GetNextSearchPos());
        ROS_INFO("G1.5 ");

        double coverage_time = TargetTour::GetPlanExecutionTime(nodeStack, node->GetMAVWaypoint(), startPos, true, false);
        double time_budget = remaining_time - coverage_time;
        ROS_INFO("G2 ");
        vector<CompoundTarget*> cts;
        cts.push_back(&ct);
        PartiallyCoverTargets(cts, time_budget, node->GetMAVWaypoint(), GetNextSearchPos());

        std::copy(newTargets.begin(), newTargets.end(), std::back_inserter(targets));

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

            CompoundTarget ct;
            for(size_t i=0; i<targets.size(); i++)
                ct.AddTarget(targets[i]);

            //TargetTour::GetTargetTour(ct.targets, node->GetMAVWaypoint(), GetNextSearchPos());
            TargetTour::GetTargetTour_Utility(ct.targets, node->GetMAVWaypoint(), GetNextSearchPos());

            //double coverage_time = TargetTour::GetPlanExecutionTime(nodeStack, node->GetMAVWaypoint(), startPos, true, false);
            double time_budget = remaining_time;

            vector<CompoundTarget*> cts;
            cts.push_back(&ct);
            PartiallyCoverTargets(cts, time_budget, node->GetMAVWaypoint(), GetNextSearchPos());
        }
    }
}

void SearchCoverageStrategy::OnReachedNode_DelayedGreedyPolicy(CNode *node, vector<TargetPolygon*> &newTargets, bool searchNode)
{
    if(searchNode)
    {
        for(size_t i=0; i<targets.size(); i++)
            targets[i]->UpdateIsVisited();
        ROS_INFO("DG 0.1 ");

        AddTargetsToComponentGenerators(newTargets, node);
        ROS_INFO("DG 0.2 ");

        targets2visit.clear();
        CleanupComponents();

        // get the targets
        gc.GetIntegratedComponents(integrated_components);
        ROS_INFO("DG1 ");

        for(size_t i=0; i<integrated_components.size(); i++)
        {
            SetCompoundTargetBoundaryFlags(integrated_components[i]);
            integrated_components[i]->SetIsCurChildFlag(node);
        }

        double coverage_time = TargetTour::GetPlanExecutionTime(nodeStack, node->GetMAVWaypoint(), startPos, true, false);
        double time_budget = remaining_time - coverage_time;

        vector<CompoundTarget*> cur_compound_targets;
        vector<CompoundTarget*> extensible_compound_targets;
ROS_INFO("DG2 ");
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

            PartiallyCoverTargets(cur_compound_targets, extra_time, node->GetMAVWaypoint(), GetNextSearchPos());


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
                        if(extensible_compound_targets[i]->cur_child)
                        {
                            extensible_compound_targets[i]->SetVisited();
                            EnqueueCompoundTarget(extensible_compound_targets[i]);
                        }
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

            PartiallyCoverTargets(cur_unselected_compounds, delay_extensibles?extra_time_delay:extra_time_cover,
                                  node->GetMAVWaypoint(), GetNextSearchPos());

        }

        std::copy(newTargets.begin(), newTargets.end(), std::back_inserter(targets));
    }
}

double SearchCoverageStrategy::LawnmowerPlanValue(std::vector<Vector<3> > &lms, const CompoundTarget* ct, bool true_value)
{
    double value = 0;
    set<CNode*> covered_cells;

    double checkPoint_dist = 0.5*NUCParam::high_res_cells*TargetPolygon::cellW;
    for(size_t i=0; i+1<lms.size(); i++)
    {
        Vector<3> p= lms[i];
        Vector<3> dir = lms[i+1]-lms[i];
        normalize(dir);
        tree->GetLeavesInRange(covered_cells, checkPoint_dist, p);

        while(D1(p,lms[i+1]) > checkPoint_dist)
        {
            p = 0.5*checkPoint_dist*dir+p;
            tree->GetLeavesInRange(covered_cells, checkPoint_dist, p);
        }
    }

    for(auto it = covered_cells.begin(); it != covered_cells.end(); ++it)
        if(ct->IsInside(*it) && (!true_value || (*it)->imgPrior>=cutoff_prob))
        {
            value += ((*it)->footPrint[0]-(*it)->footPrint[2])*((*it)->footPrint[0]-(*it)->footPrint[2]);
        }

    return value;
}

void SearchCoverageStrategy::PartiallyCoverTargets(vector<CompoundTarget*> &cts, const double budget,
                                                   TooN::Vector<3> cur_pos, TooN::Vector<3> next_pos)
{
    vector<Vector<3> > best_partial_lm;
    double best_partial_value = 0;
    double partial_value_true=0;

    for(size_t i=0; i<cts.size(); i++)
    {
        CompoundTarget &ct = *cts[i];

        ct.SetIgnored();
        ct.SetVisited();

        vector<Vector<3> > partial_lm;
        double partial_cost = 0;
        double partial_value = 0;

        ct.GetLawnmowerPlan(partial_lm);
        partial_cost = TargetTour::GetPlanExecutionTime(partial_lm, cur_pos, next_pos, true, true);

        bool lastPoped_flag = false;
        Vector<3> lastPoped;

        while(partial_cost > budget && !partial_lm.empty())
        {
            lastPoped_flag = true;
            lastPoped = partial_lm.back();
            partial_lm.pop_back();

            partial_cost = TargetTour::GetPlanExecutionTime(partial_lm, cur_pos, next_pos, true, true);
        }

        if(lastPoped_flag && !partial_lm.empty())
        {
            size_t count = 0;
            size_t l = partial_lm.size();
            partial_lm.push_back(lastPoped);
            partial_cost = TargetTour::GetPlanExecutionTime(partial_lm, cur_pos, next_pos, true, true);
            while(partial_cost > budget)
            {
                partial_lm[l] += 0.5*(partial_lm[l-1]-partial_lm[l]);
                partial_cost = TargetTour::GetPlanExecutionTime(partial_lm, cur_pos, next_pos, true, true);

                if(count++ > 5)
                {
                    partial_lm.pop_back();
                    partial_cost = TargetTour::GetPlanExecutionTime(partial_lm, cur_pos, next_pos, true, true);
                    break;
                }
            }
        }


        partial_value = LawnmowerPlanValue(partial_lm, &ct);

        if(partial_value > best_partial_value)
        {
            partial_value_true = LawnmowerPlanValue(partial_lm, &ct, true);
            best_partial_value = partial_value;
            best_partial_lm.clear();
            copy(partial_lm.begin(), partial_lm.end(), back_inserter(best_partial_lm));
        }
    }

    high_res_coverage += best_partial_value;
    high_res_coverage_true += partial_value_true;
    copy(best_partial_lm.begin(), best_partial_lm.end(), back_inserter(target_lms));
    ROS_INFO("Partial Coverage Value: %.2f", best_partial_value);
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



void SearchCoverageStrategy::EnqueueCompoundTarget(const CompoundTarget *ct)
{
    high_res_coverage += ct->value;
    high_res_coverage_true += ct->GetTrueValue();
    ct->GetLawnmowerPlan(target_lms);
}

void SearchCoverageStrategy::SetupCostsValues_NoExtensibleTarget(std::vector<CompoundTarget *> &cur_targets,
                                              std::vector<CompoundTarget *> &extensible_targets, CNode* cur_node)
{
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
            double c = TargetTour::GetTargetTour(v.targets, nds[k]->GetMAVWaypoint(), nds[k]->GetMAVWaypoint());
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
        v.cost = TargetTour::GetTargetTour(v.targets, cur_node->GetMAVWaypoint(), cur_node->GetMAVWaypoint());
    }
}

void SearchCoverageStrategy::SetupCostsValues_WithExtensibleTarget(std::vector<CompoundTarget *> &cur_targets,
                                              std::vector<CompoundTarget *> &extensible_targets, CNode* cur_node, bool delay)
{
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
                double c = TargetTour::GetTargetTour(v.targets, nds[k]->GetMAVWaypoint(), nds[k]->GetMAVWaypoint());
                if(minCost > c)
                {
                    minCost = c;
                    minNode = nds[k];
                }
            }
        }
        else
        {
            minCost = TargetTour::GetTargetTour(v.targets, cur_node->GetMAVWaypoint(), cur_node->GetMAVWaypoint());
            minNode = cur_node;
        }

        //ROS_INFO("Setting cost: %f cur_child:%d delay:%d #nds:%lu", minCost, v.cur_child, delay, nds.size());
        v.startNode = minNode;
        v.cost = minCost;
    }

    for(size_t i=0; i < cur_targets.size(); i++)
    {
        CompoundTarget &v = *cur_targets[i];

        v.CalculateValue();
        v.startNode = cur_node;
        v.cost = TargetTour::GetTargetTour(v.targets, cur_node->GetMAVWaypoint(), cur_node->GetMAVWaypoint());
    }
}

void SearchCoverageStrategy::SeparateCompoundTargets(const vector<CompoundTarget*> &all_targets, CNode* cur_search_node,
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

bool SearchCoverageStrategy::NeighboursNode(const CNode *n1, const CNode *n2) const
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
    ScalarField::GetInstance()->glDraw();

    if(drawFootprints && hi_res_waypoints.size() > 2)
    {
        double half_fp = hi_res_waypoints[0][2] * tan(NUCParam::FOV*0.5);

        glColor4f(1,1,0,0.5);
        glBegin(GL_QUADS);
        for(size_t i=0; i+1<hi_res_waypoints.size(); i++)
        {
            Vector<3> p1 = hi_res_waypoints[i];
            Vector<3> p2 = hi_res_waypoints[i+1];

            double dist = D2(p1,p2);
            do
            {
                Vector<3> dir = p2-p1;
                normalize(dir);
                Vector<3> orthDir = makeVector(-dir[1], dir[0], dir[2]);

                Vector<3> p[4];
                p[0] = p1 + ( half_fp*dir+half_fp*orthDir);
                p[1] = p1 + (-half_fp*dir+half_fp*orthDir);
                p[2] = p1 + (-half_fp*dir-half_fp*orthDir);
                p[3] = p1 + ( half_fp*dir-half_fp*orthDir);

                glVertex(p[0]);
                glVertex(p[1]);
                glVertex(p[2]);
                glVertex(p[3]);

                p1 += dir;
                dist = D2(p1,p2);

            }while(dist > half_fp*half_fp);
        }

        glEnd();
    }

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

//    if(nodeStack.size() > 2)
//    {
//        glColor3f(0.5,0.6,0.6);
//        glLineWidth(4);
//        glBegin(GL_LINES);
//        for(unsigned int i=0; i<nodeStack.size()-1;i++)
//        {
//            TooN::Vector<3> p1 = nodeStack[i+1]->GetMAVWaypoint();
//            TooN::Vector<3> p2 = nodeStack[i]->GetMAVWaypoint();

//            glVertex3f(p1[0],p1[1],p1[2]);
//            glVertex3f(p2[0],p2[1],p2[2]);
//        }
//        glEnd();
//    }

    for_each(search_grid.begin(), search_grid.end(), boost::bind(&CNode::glDraw,_1));
    //gc.glDraw();

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

    for(auto tg: targets)
        tg->glDraw();

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

Rect SearchCoverageStrategy::GetCourseSurveyFootprint()
{
    if(!search_grid.empty())
        return search_grid.front()->GetFootPrint();
    else
        return Rect();
}

CNode * SearchCoverageStrategy::GetNode(int i, int j) const
{
    int n = i*s+j;
    if(n >=0 && n < (int)grid.size())
        return grid[n];
    else
    {
        //ROS_WARN("GetNode: out of boundary access ...");
        return NULL;
    }
}

bool SearchCoverageStrategy::InSearchGridBoundary(int i, int j) const
{
    if(i < 0 || j < 0 || i >= NUCParam::lm_tracks || j >= NUCParam::lm_tracks)
    {
        return false;
    }

    return true;
}

CNode * SearchCoverageStrategy::GetSearchNode(int i, int j) const
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
    if(key['4'])
    {
        //drawFootprints = !drawFootprints;
        GenerateEnvironment(true);
        updateKey = false;
    }
    else if(key['6'])
    {
        NUCParam::bypass_controller = true;
        updateKey = false;
    }
    else if(key['7'])
    {
        if(CNode::drawing_mode == CNode::DrawingMode::gp_true_f)
            CNode::drawing_mode = CNode::DrawingMode::gp_f;
        else if(CNode::drawing_mode == CNode::DrawingMode::gp_f)
            CNode::drawing_mode = CNode::DrawingMode::gp_var;
        else if(CNode::drawing_mode == CNode::DrawingMode::gp_var)
            CNode::drawing_mode = CNode::DrawingMode::gp_f_var;
        else if(CNode::drawing_mode == CNode::DrawingMode::gp_f_var)
            CNode::drawing_mode = CNode::DrawingMode::Interesting;
        else
            CNode::drawing_mode = CNode::DrawingMode::gp_true_f;


        updateKey = false;
    }
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

void SearchCoverageStrategy::FindClusters(bool incremental, vector<TargetPolygon*> &newTargets, CNode* ancester)
{
    static int cn = newTargets.size()-1;
    if(!incremental)
        cn = -1;

    size_t old_cluster_n = cn+1;

    auto InAncester = [=](CNode* cnd){return ancester?IN(cnd->GetMAVWaypoint(), ancester->GetFootPrint()):true;};

    for(int i=0; i<s; i++)
       for(int j=0; j<s; j++)
       {
           if(!incremental || (GetNode(i,j)->label==-1 && InAncester(GetNode(i,j))))
           {
                GetNode(i,j)->extra_info = false;
                GetNode(i,j)->label = -1;
           }
           else
           {
                GetNode(i,j)->extra_info = true;
           }
       }

    for(int i=0; i<s; i++)
    {
        for(int j=0; j<s; j++)
        {
            if(GetNode(i,j)->extra_info || GetNode(i,j)->p_X < 0.99 || GetNode(i,j)->visited || !GetNode(i,j)->ancestor_visited || !InAncester(GetNode(i,j)))
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
                    if( ne && !ne->extra_info && ne->p_X > 0.95 && !ne->visited && ne->ancestor_visited && InAncester(GetNode(i,j)))
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

        TargetPolygon *t = new TargetPolygon(v, visitedNodes.back(),[&](int i_, int j_){ return this->GetNode(i_,j_);});
        if(!t->lm.empty())
            newTargets.push_back(t);
        else
            delete t;
    }
}
