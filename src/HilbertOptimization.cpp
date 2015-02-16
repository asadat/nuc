#include "HilbertOptimization.h"
#include <GL/glut.h>
#include "ros/ros.h"
#include "NUCParam.h"

HilbertOptimization::HilbertOptimization(CNode *root, TooN::Vector<3> init_pos, TooN::Vector<3> end_pos):
    HilbertStrategy(root)
{

    draw_i_path = 0;

    startNode = new GNode(init_pos, 0);
    endNode = new GNode(end_pos, 0);

    //generate forward links
    for(unsigned int i=0; i <= lastDepth; i++)
    {
        startNode->AddNext(hilbert[i][0]);
        for(unsigned int j=0; j < hilbert[i].size()-1; j++)
        {
            //Add linke to next node
            hilbert[i][j]->GetGNode()->AddNext(hilbert[i][j+1]);

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

    ros::Time s = ros::Time::now();
    ROS_INFO("Optimizer started \n");
    optimizer = new PathOptimization(startNode);
    optimizer->FindBestPath(endNode, NUCParam::pathCost, p);
    delete optimizer;

    ROS_INFO("dtime: %f", (ros::Time::now()-s).toSec());
    p.PrintOut();

}

HilbertOptimization::~HilbertOptimization()
{

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
        ROS_INFO("Node Reward: %f", p.path[i]->NodeReward());
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

}
