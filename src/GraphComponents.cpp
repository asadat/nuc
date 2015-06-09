#include "GraphComponents.h"
#include <algorithm>
#include <GL/glut.h>
#include "ros/ros.h"

GraphComponents::GraphComponents()
{

}

GraphComponents::~GraphComponents()
{
    Clear();
}

node* GraphComponents::GetNode(TargetPolygon *tp)
{
    node* n = NULL;
    map<TargetPolygon*, node*>::const_iterator it = target2node.find(tp);
    if(it == target2node.end())
    {
        //ROS_INFO("Didn't found the node. #nodes: %d", nodes.size());

        n = new node();
        n->tp = tp;
        nodes.push_back(n);
        target2node.insert(pair<TargetPolygon*, node*>(tp, n));

    }
    else
    {
        n = target2node[tp];
    }

   // ROS_INFO("#nodes: %d", nodes.size());

    return n;
}

void GraphComponents::AddEdge(TargetPolygon *tp1, TargetPolygon *tp2, bool virtual_edge)
{
    //ROS_INFO("Adding Edge.....");

    node* n1 = GetNode(tp1);
    node* n2 = GetNode(tp2);

    if(!virtual_edge)
    {
        pair<multimap<node*,node*>::iterator, multimap<node*,node*>::iterator> r_it = edges.equal_range(n1);
        for(multimap<node*,node*>::iterator it=r_it.first; it != r_it.second; it++)
        {
            if(it->first == n1 && it->second == n2)
                return;
        }

        edges.insert(pair<node*,node*>(n1,n2));
        edges.insert(pair<node*,node*>(n2,n1));
    }
    else
    {
        pair<multimap<node*,node*>::iterator, multimap<node*,node*>::iterator> r_it = virtual_edges.equal_range(n1);
        for(multimap<node*,node*>::iterator it=r_it.first; it != r_it.second; it++)
        {
            if(it->first == n1 && it->second == n2)
                return;
        }

        virtual_edges.insert(pair<node*,node*>(n1,n2));
        virtual_edges.insert(pair<node*,node*>(n2,n1));

    }

}

void GraphComponents::GetConnectedComponents(vector<vector<TargetPolygon *> *> &components)
{
    for(size_t i=0; i<nodes.size(); i++)
        nodes[i]->visited = false;

    for(size_t i=0; i<nodes.size(); i++)
    {
        if(!nodes[i]->visited)
        {
            vector<TargetPolygon*> * cmp = new vector<TargetPolygon*>();
            DFS(nodes[i], cmp);
            components.push_back(cmp);
        }
    }

    //ROS_INFO("#nodes: %d", nodes.size());
}

void GraphComponents::DFS(node *root, vector<TargetPolygon *> *cmp)
{
    root->visited = true;
    cmp->push_back(root->tp);

    pair<multimap<node*,node*>::iterator, multimap<node*,node*>::iterator> r_it = edges.equal_range(root);
    for(multimap<node*,node*>::iterator it=r_it.first; it != r_it.second; it++)
    {
        if(!it->second->visited)
        {
            DFS(it->second, cmp);
        }
    }

    r_it = virtual_edges.equal_range(root);
    for(multimap<node*,node*>::iterator it=r_it.first; it != r_it.second; it++)
    {
        if(!it->second->visited)
        {
            DFS(it->second, cmp);
        }
    }
}

void GraphComponents::Clear()
{
   // ROS_INFO("CLEARED");
    target2node.clear();
    edges.clear();
    virtual_edges.clear();
    while(!nodes.empty())
    {
        node * n = nodes.back();
        nodes.pop_back();
        delete n;
    }
}

void GraphComponents::glDraw()
{
 //   ROS_INFO("#nodes: %d", nodes.size());

    multimap<node*, node*>::iterator it;
    for(it = edges.begin(); it != edges.end(); it++)
    {
        Vector<3> p1 = (it->first)->tp->GetCenter();
        Vector<3> p2 = (it->second)->tp->GetCenter();
        glLineWidth(5);
        glColor3f(1,0,0);
        glBegin(GL_LINES);
        glVertex3f(p1[0], p1[1], p1[2]+.2);
        glVertex3f(p2[0], p2[1], p2[2]+.2);
        glEnd();
    }

    for(it = virtual_edges.begin(); it != virtual_edges.end(); it++)
    {
        Vector<3> p1 = (it->first)->tp->GetCenter();
        Vector<3> p2 = (it->second)->tp->GetCenter();
        glLineWidth(5);
        glColor3f(1,0,0);
        glBegin(GL_LINES);
        glVertex3f(p1[0], p1[1], p1[2]+.2);
        glVertex3f(p2[0], p2[1], p2[2]+.2);
        glEnd();
    }

    for(size_t i=0; i<nodes.size();i++)
    {
        Vector<3> p = nodes[i]->tp->GetCenter();
        glPointSize(7);
        glColor3f(1,0,1);
        glBegin(GL_POINTS);
        glVertex3f(p[0], p[1], p[2]+.2);
        glEnd();
    }
}
