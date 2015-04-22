#ifndef TSP_
#define TSP_

#include "TooN/TooN.h"
#include <vector>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/metric_tsp_approx.hpp>

using namespace TooN;
using namespace std;
using namespace boost;

struct Entity
{
    Vector<3,double> pos;
    bool start;
    bool end;
    int nodeIdx;
};

class TSP
{
    public:

        typedef adjacency_matrix<boost::undirectedS, boost::no_property,
            boost::property <boost::edge_weight_t, double,
            boost::property<boost::edge_index_t, int> > > Boost_Graph;

        typedef adjacency_matrix<undirectedS, no_property,
            property <edge_weight_t, double> > VertexListGraph;

        typedef graph_traits<VertexListGraph>::vertex_descriptor Vertex;
        typedef graph_traits<VertexListGraph>::edge_descriptor Edge;

        typedef property_map<VertexListGraph, edge_weight_t>::type WeightMap;
        typedef property_map<VertexListGraph, vertex_index_t>::type VertexMap;

        typedef typename graph_traits<VertexListGraph>::vertex_iterator VItr;

        TSP();
        ~TSP();

        void GetShortestPath(vector<Entity* >&, vector<Entity* >&);

    private:

        void TwoOptOptimization(vector<Entity* > &path);
        map<Vertex, Entity*> map_vertices_to_entities(vector<Entity*> entities, VertexListGraph &g);
        void create_connected_graph(VertexListGraph &g, WeightMap wmap, std::map<Vertex,Entity*> vmap);
};

#endif
