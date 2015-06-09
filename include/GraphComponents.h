#ifndef _GRAPH_COMPONENTS_
#define _GRAPH_COMPONENTS_

#include <map>
#include <vector>
#include "TargetPolygon.h"

using namespace std;
struct node
{
    TargetPolygon* tp;
    bool visited;
};

class GraphComponents
{
    public:
        GraphComponents();
        ~GraphComponents();

        void AddEdge(TargetPolygon * tp1, TargetPolygon *tp2, bool virtual_edge);
        void GetConnectedComponents(vector<vector<TargetPolygon*> *> &components);
        void Clear();
        void glDraw();

    private:
        void DFS(node* root, vector<TargetPolygon*> * cmp);
        node* GetNode(TargetPolygon* tp);
        map<TargetPolygon*, node*> target2node;
        multimap<node*, node*> edges;
        multimap<node*, node*> virtual_edges;
        vector<node*> nodes;
};
#endif
