

//
//=======================================================================
// Copyright (c) 2004 Kristopher Beevers
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
//


#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <utility>
#include <vector>
#include <list>
#include <iostream>
#include <math.h>    // for sqrt
#include <time.h>

using namespace boost;
using namespace std;


// auxiliary types
struct location
{
  float y, x; // lat, long
};
struct my_float {float v; explicit my_float(float v = float()): v(v) {}};
typedef my_float cost;
ostream& operator<<(ostream& o, my_float f) {return o << f.v;}
my_float operator+(my_float a, my_float b) {return my_float(a.v + b.v);}
bool operator==(my_float a, my_float b) {return a.v == b.v;}
bool operator<(my_float a, my_float b) {return a.v < b.v;}

template <class Name, class LocMap>
class city_writer {
public:
  city_writer(Name n, LocMap l, float _minx, float _maxx,
              float _miny, float _maxy,
              unsigned int _ptx, unsigned int _pty)
    : name(n), loc(l), minx(_minx), maxx(_maxx), miny(_miny),
      maxy(_maxy), ptx(_ptx), pty(_pty) {}
  template <class Vertex>
  void operator()(ostream& out, const Vertex& v) const {
    float px = 1 - (loc[v].x - minx) / (maxx - minx);
    float py = (loc[v].y - miny) / (maxy - miny);
    out << "[label=\"" << name[v] << "\", pos=\""
        << static_cast<unsigned int>(ptx * px) << ","
        << static_cast<unsigned int>(pty * py)
        << "\", fontsize=\"11\"]";
  }
private:
  Name name;
  LocMap loc;
  float minx, maxx, miny, maxy;
  unsigned int ptx, pty;
};

template <class WeightMap>
class time_writer {
public:
  time_writer(WeightMap w) : wm(w) {}
  template <class Edge>
  void operator()(ostream &out, const Edge& e) const {
    out << "[label=\"" << wm[e] << "\", fontsize=\"11\"]";
  }
private:
  WeightMap wm;
};


// euclidean distance heuristic
template <class Graph, class CostType, class LocMap>
class distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
  typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
  distance_heuristic(LocMap l, Vertex goal)
    : m_location(l), m_goal(goal) {}
  CostType operator()(Vertex u)
  {
    float dx = m_location[m_goal].x - m_location[u].x;
    float dy = m_location[m_goal].y - m_location[u].y;
    return CostType(::sqrt(dx * dx + dy * dy));
  }
private:
  LocMap m_location;
  Vertex m_goal;
};


struct found_goal {}; // exception for termination

// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
  astar_goal_visitor(Vertex goal) : m_goal(goal) {}
  template <class Graph>
  void examine_vertex(Vertex u, Graph&) {
    if(u == m_goal)
      throw found_goal();
  }
private:
  Vertex m_goal;
};


int main(int, char **)
{
  
  // specify some types
  typedef adjacency_list<listS, vecS, undirectedS, no_property,
    property<edge_weight_t, cost> > mygraph_t;
  typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
  typedef mygraph_t::vertex_descriptor vertex;
  typedef mygraph_t::edge_descriptor edge_descriptor;
  typedef std::pair<int, int> edge;
  
  // specify data
  enum nodes {
    Troy, LakePlacid, Plattsburgh, Massena, Watertown, Utica,
    Syracuse, Rochester, Buffalo, Ithaca, Binghamton, Woodstock,
    NewYork, N
  };
  const char *name[] = {
    "Troy", "Lake Placid", "Plattsburgh", "Massena",
    "Watertown", "Utica", "Syracuse", "Rochester", "Buffalo",
    "Ithaca", "Binghamton", "Woodstock", "New York"
  };
  location locations[] = { // lat/long
    {42.73f, 73.68f}, {44.28f, 73.99f}, {44.70f, 73.46f},
    {44.93f, 74.89f}, {43.97f, 75.91f}, {43.10f, 75.23f},
    {43.04f, 76.14f}, {43.17f, 77.61f}, {42.89f, 78.86f},
    {42.44f, 76.50f}, {42.10f, 75.91f}, {42.04f, 74.11f},
    {40.67f, 73.94f}
  };
  edge edge_array[] = {
    edge(Troy,Utica), edge(Troy,LakePlacid),
    edge(Troy,Plattsburgh), edge(LakePlacid,Plattsburgh),
    edge(Plattsburgh,Massena), edge(LakePlacid,Massena),
    edge(Massena,Watertown), edge(Watertown,Utica),
    edge(Watertown,Syracuse), edge(Utica,Syracuse),
    edge(Syracuse,Rochester), edge(Rochester,Buffalo),
    edge(Syracuse,Ithaca), edge(Ithaca,Binghamton),
    edge(Ithaca,Rochester), edge(Binghamton,Troy),
    edge(Binghamton,Woodstock), edge(Binghamton,NewYork),
    edge(Syracuse,Binghamton), edge(Woodstock,Troy),
    edge(Woodstock,NewYork)
  };
  unsigned int num_edges = sizeof(edge_array) / sizeof(edge);
  cost weights[] = { // estimated travel time (mins)
    my_float(96), my_float(134), my_float(143), my_float(65), my_float(115), my_float(133), my_float(117), my_float(116), my_float(74), my_float(56),
    my_float(84), my_float(73), my_float(69), my_float(70), my_float(116), my_float(147), my_float(173), my_float(183), my_float(74), my_float(71), my_float(124)
  };
  
  
  // create graph
  mygraph_t g(N);
  WeightMap weightmap = get(edge_weight, g);
  for(std::size_t j = 0; j < num_edges; ++j) {
    edge_descriptor e; bool inserted;
    boost::tie(e, inserted) = add_edge(edge_array[j].first,
                                       edge_array[j].second, g);
    weightmap[e] = weights[j];
  }
  
  
  // pick random start/goal
  boost::minstd_rand gen(static_cast<uint32_t>(time(0)));
  vertex start = gen() % num_vertices(g);
  vertex goal = gen() % num_vertices(g);
  
  
  cout << "Start vertex: " << name[start] << endl;
  cout << "Goal vertex: " << name[goal] << endl;
  
  vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
  vector<cost> d(num_vertices(g));

  boost::property_map<mygraph_t, boost::vertex_index_t>::const_type
    idx = get(boost::vertex_index, g);

  try {
    // call astar named parameter interface
    astar_search
      (g, start,
       distance_heuristic<mygraph_t, cost, location*>
        (locations, goal),
       predecessor_map(make_iterator_property_map(p.begin(), idx)).
       distance_map(make_iterator_property_map(d.begin(), idx)).
       visitor(astar_goal_visitor<vertex>(goal)).distance_inf(my_float((std::numeric_limits<float>::max)())));
  
  
  } catch(found_goal) { // found a path to the goal
    list<vertex> shortest_path;
    for(vertex v = goal;; v = p[v]) {
      shortest_path.push_front(v);
      if(p[v] == v)
        break;
    }
    cout << "Shortest path from " << name[start] << " to "
         << name[goal] << ": ";
    list<vertex>::iterator spi = shortest_path.begin();
    cout << name[start];
    for(++spi; spi != shortest_path.end(); ++spi)
      cout << " -> " << name[*spi];
    cout << endl << "Total travel time: " << d[goal] << endl;
    return 0;
  }
  
  cout << "Didn't find a path from " << name[start] << "to"
       << name[goal] << "!" << endl;
  return 0;
  
}
