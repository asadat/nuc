#include "HilbertOptimization.h"
#include <GL/glut.h>
#include "ros/ros.h"
#include "NUCParam.h"

HilbertOptimization::HilbertOptimization(CNode *root, TooN::Vector<3> init_pos, TooN::Vector<3> end_pos):
    HilbertStrategy(root)
{

    draw_i_path = 0;
    startNode = new GNode(init_pos, 0);
    startNode->maxRewardToGoal = 0;
    endNode = new GNode(end_pos, 0);
    endNode->maxRewardToGoal = 0;

    greedyPath = NULL;

    CreateGraph();
    greedyPath = FindGreedyPath();

    ROS_INFO("Greedy Path Cost: %f Reward: %f", greedyPath->cost, greedyPath->reward);
    FindPath();

}

HilbertOptimization::~HilbertOptimization()
{
    GNode::Path * gp = new GNode::Path();

}

void HilbertOptimization::CreateGraph()
{
    //generate forward links
    for(unsigned int i=0; i <= lastDepth; i++)
    {
        startNode->AddNext(hilbert[i][0]);
        for(unsigned int j=0; j < hilbert[i].size()-1; j++)
        {
            //Add linke to next node
            hilbert[i][j]->GetGNode()->AddNext(hilbert[i][j+1]);

            if(i == lastDepth)
            {
                hilbert[i][j]->GetGNode()->leaf = true;

                if(j == hilbert[i].size()-2)
                {
                    hilbert[i][j+1]->GetGNode()->leaf = true;
                }
            }

            // linke to uncle
            if(j%4 == 3)
            {
                hilbert[i][j]->GetGNode()->AddNext(hilbert[i][j+1]->GetParent());

                if(j%16 == 15)
                {
                    hilbert[i][j]->GetGNode()->AddNext(hilbert[i][j+1]->GetParent()->GetParent());
                }
            }

            // link to nephew
            if(j>0 && j%4==0)
            {
                hilbert[i][j-1]->GetParent()->GetGNode()->AddNext(hilbert[i][j]);

                if(j>0 && j%16==0)
                {
                    if(hilbert[i][j-1]->GetParent()->GetParent())
                        hilbert[i][j-1]->GetParent()->GetParent()->GetGNode()->AddNext(hilbert[i][j]);
                }
            }
        }

        hilbert[i].back()->GetGNode()->AddNext(endNode);
    }

    hilbert[lastDepth].back()->GetGNode()->maxRewardToGoal = hilbert[lastDepth].back()->CoverageReward();

    for(unsigned int k = hilbert[lastDepth].size()-1; k>0; k--)
    {
        hilbert[lastDepth][k-1]->GetGNode()->maxRewardToGoal = hilbert[lastDepth][k]->GetGNode()->maxRewardToGoal +
                hilbert[lastDepth][k-1]->GetGNode()->NodeReward();
    }
}

void HilbertOptimization::FindPath()
{
    ros::Time s = ros::Time::now();
    ROS_INFO("Optimizer started \n");
    optimizer = new PathOptimization(startNode);
    optimizer->FindBestPath(endNode, NUCParam::pathCost, greedyPath->reward, p);
    ROS_INFO("dtime: %f", (ros::Time::now()-s).toSec());
    p.PrintOut();
}

GNode::Path* HilbertOptimization::FindGreedyPath()
{
    GNode::Path* gp = new GNode::Path();
    gp->path.push_back(startNode);
    gp->path.push_back(hilbert[0][0]->GetGNode());
    gp->path.push_back(endNode);

    while(true)
    {
        double r = -1;
        int replace_idx = -1;
        for(int i=1; i < gp->path.size()-1; i++)
        {
            if(gp->path[i]->cnode->IsLeaf())
                continue;

            if(gp->path[i]->greedy_count > 0)
            {
                gp->path[i]->greedy_count--;
                continue;
            }

//            double mrg_ben = 0;
//            double mrg_cst = 0;
//            mrg_ben = gp->ReplaceNodeReward(gp->path[i], gp->path[i]->cnode->ordered_children, NUCParam::pathCost, mrg_cst);
//            double util =

            double new_r = gp->ReplaceNodeReward(gp->path[i], gp->path[i]->cnode->ordered_children, NUCParam::pathCost);
            if( new_r > r)
            {
                r = new_r;
                replace_idx = i;
            }
        }

        if(replace_idx >= 0)
        {
            gp->ReplaceNode(gp->path[replace_idx], gp->path[replace_idx]->cnode->ordered_children);
            for(unsigned int k=0; k< gp->path[replace_idx]->cnode->ordered_children.size(); k++)
            {
                gp->path[replace_idx]->cnode->ordered_children[k]->GetGNode()->greedy_count = 0;
            }
        }
        else
        {
            return gp;
        }
    }

}

CNode* HilbertOptimization::GetNextNode()
{
    static int i=0;

    if(i >= p.path.size()-2)
    {
        return NULL;
    }
    else
    {
        i++;
        //ROS_INFO("Node Reward: %f", p.path[i]->NodeReward());
        return p.path[i]->GetCNode();

    }

}

void HilbertOptimization::hanldeKeyPressed(map<unsigned char, bool> &key, bool &updateKey)
{
//    if(key[']'])
//    {
//        draw_i_path++;
//        updateKey = false;
//        if(endNode && draw_i_path>=0 && draw_i_path<endNode->bestPaths.size())
//        {
//            ROS_INFO("#%d  reward:%f",draw_i_path, endNode->bestPaths[draw_i_path]->reward);
//        }
//    }
//    else if(key['['])
//    {
//        draw_i_path--;
//        updateKey = false;
//        if(endNode && draw_i_path>=0 && draw_i_path<endNode->bestPaths.size())
//        {
//            ROS_INFO("#%d  reward:%f",draw_i_path, endNode->bestPaths[draw_i_path]->reward);
//        }
//    }
}

void HilbertOptimization::glDraw()
{
    //glBegin(GL_LINES);
    //startNode->glDraw();
    //glEnd();

//    if(endNode && draw_i_path>=0 && draw_i_path<endNode->bestPaths.size())
//    {
//        GNode::Path * ps = endNode->bestPaths[draw_i_path];

//        glLineWidth(7);
//        glBegin(GL_LINES);

//        for(int i=1; i<ps->path.size()-2;++i)
//        {
//            double r = ps->path[i+1]->NodeReward();

//            assert(r>=0);
//            assert(r<=1);

//            glColor3f(0,r,1-r);
//            TooN::Vector<3> p1 = ps->path[i]->cnode->GetPos();
//            TooN::Vector<3> p2 = ps->path[i+1]->cnode->GetPos();

//            glVertex3f(p1[0],p1[1],p1[2]);
//            glVertex3f(p2[0],p2[1],p2[2]);
//        }

//        glEnd();

//    }

    if(greedyPath)
    {
        glLineWidth(7);
        glBegin(GL_LINES);

        for(int i=0; i<greedyPath->path.size()-1;++i)
        {
            double r = greedyPath->path[i+1]->NodeReward()/(CNode::maxDepth+1);

            //assert(r>=0);
            //assert(r<=1);

            glColor3f(0,r,1-r);
            TooN::Vector<3> p1 = greedyPath->path[i]->cnode->GetPos();
            TooN::Vector<3> p2 = greedyPath->path[i+1]->cnode->GetPos();

            glVertex3f(p1[0],p1[1],p1[2]);
            glVertex3f(p2[0],p2[1],p2[2]);
        }

        glEnd();

    }

}
