#include "TSP.h"
#include <stdio.h>

double d(Vector<3> a, Vector<3> b)
{
    return sqrt((a-b)*(a-b));
}

TSP::TSP()
{
}

TSP::~TSP()
{
}

void TSP::GetShortestPath(vector<Entity* > &to_visit, vector<Entity* >& tsplist)
{
    vector<Vertex> shortest_path_vect;
    VertexListGraph g( to_visit.size());
    WeightMap weight_map(get(edge_weight, g));

    //populate map from vertices to Triangle3D structs
    std::map<Vertex,Entity*> verts_to_ent =  map_vertices_to_entities(to_visit, g);

    create_connected_graph( g, weight_map, verts_to_ent);

    //solve metric tsp
    metric_tsp_approx_tour(g, back_inserter(shortest_path_vect));

    //begin()+1 excludes the first vertex and end()-1 excludes the last, since they're both the dummy vertex
    for (vector<Vertex>::iterator itr = shortest_path_vect.begin(); itr != shortest_path_vect.end(); ++itr)
    {
        tsplist.push_back( verts_to_ent[*itr] );
    }

   // TwoOptOptimization(tsplist);

}

void TSP::TwoOptOptimization(vector<Entity *> &path)
{
    for(size_t i=0; i+3<path.size(); i++)
    {
        if(d(path[i]->pos,path[i+2]->pos) + d(path[i+2]->pos,path[i+1]->pos) + d(path[i+1]->pos,path[i+3]->pos) <
                d(path[i]->pos,path[i+1]->pos) + d(path[i+1]->pos,path[i+2]->pos) + d(path[i+2]->pos,path[i+3]->pos))
        {
            Entity* i_2 = path[i+2];
            path.erase(path.begin()+i+2);
            path.insert(path.begin()+i+1, i_2);
        }
    }
}

map<TSP::Vertex, Entity*> TSP::map_vertices_to_entities(vector<Entity*> entities, VertexListGraph &g)
{
    std::map<Vertex, Entity*> v_pmap;

    VItr vi, ve;
    int idx(0);
    for (boost::tie(vi, ve) = vertices(g); vi != ve; ++vi)
    {
        Vertex v(*vi);
        v_pmap[v] = entities[idx];
        idx++;
    }

    return v_pmap;
}

void TSP::create_connected_graph(VertexListGraph &g, WeightMap wmap, std::map<Vertex,Entity*> vmap)
{
    Edge e;
    bool inserted;

    pair<VItr, VItr> verts(vertices(g));
    for (VItr src(verts.first); src != verts.second; src++)
    {
        for (VItr dest(src); dest != verts.second; dest++)
        {
            if (dest != src)
            {
                double weight = 0;


                //the weights to and from the dummy vertex (indexes NULL) will be left as zero
                if (vmap.find(*src) != vmap.end() && vmap.find(*dest) != vmap.end())
                {
                    Vector<3> src_center = ( vmap[*src]->pos );
                    Vector<3> dest_center =( vmap[*dest]->pos );

                    weight = sqrt((src_center-dest_center)*(src_center-dest_center));
                }
                else
                {
                    if(vmap.find(*src)!=vmap.end())
                    {
                        if(!vmap[*src]->start && !vmap[*src]->end )
                        {
                            weight = 99999999;
                        }
                    }

                    if(vmap.find(*dest)!=vmap.end())
                    {
                        if(!vmap[*dest]->start && !vmap[*dest]->end)
                        {
                            weight = 999999999;
                        }
                    }
                }

                boost::tie(e, inserted) = add_edge(*src, *dest, g);
                wmap[e] = weight;
            }
        }
    }   
}
