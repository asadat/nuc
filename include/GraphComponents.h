#ifndef _GRAPH_COMPONENTS_
#define _GRAPH_COMPONENTS_

#include <map>
#include <vector>

using namespace std;

class TargetPolygon;
class CompoundTarget;

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
        void AddNode(TargetPolygon * tp);
        void GetConnectedComponents(vector<CompoundTarget*> &components);
        void GetIntegratedComponents(vector<CompoundTarget*> &components);
        void Clear();
        void glDraw();

    private:
        void DFS(node* root, CompoundTarget * cmp);
        void Integrating_DFS(node* root, TargetPolygon* target);
        node* GetNode(TargetPolygon* tp);
        map<TargetPolygon*, node*> target2node;
        multimap<node*, node*> edges;
        multimap<node*, node*> virtual_edges;
        vector<node*> nodes;
};
#endif
