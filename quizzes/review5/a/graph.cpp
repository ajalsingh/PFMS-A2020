#include <set>
#include <queue>
#include "graph.h"
#include <string>
#include <map>
#include <algorithm>


void Graph::addVertex(vertex v) {
  //Insert a vertex with no edges
  weightedGraph_.insert({v, edges_t()});
}

bool Graph::hasVertex(vertex v) {
  //Check if the vertex v, exists in the graph
  auto i = weightedGraph_.find(v);
  if (i != weightedGraph_.end()) {
    return true;
  }
  return false;
}

void Graph::addEdge(vertex u, vertex v, weight w) {
  //Assumes that u & v have already been added to the graph
  //We need to record the same edge twice as this is an undirected graph
  weightedGraph_.at(u).insert({v, w}); //Inserting an edge between u and v, with weight w
  weightedGraph_.at(v).insert({u, w}); //Inserting an edge between v and u, with weight w
}


std::vector<Graph::vertex> Graph::getVertices(void) {
  //Iterate through the weightedGraph_ and push back each vertex to the vertices vector
  std::vector<Graph::vertex> verticies;
  for (graph_t::iterator i = weightedGraph_.begin(); i != weightedGraph_.end(); i++)
  {
    verticies.push_back(i->first);
  }

  return std::vector<Graph::vertex>(verticies);
}

std::vector<Graph::vertex> Graph::getNeighbours(vertex v) {
  std::vector<Graph::vertex> neighbours;
  auto i = weightedGraph_.find(v);
  if (i != weightedGraph_.end()) {
    edges_t edges = i->second;
    for (auto neighbour: edges) {
      neighbours.push_back(neighbour.first);
    }
  }
  return std::vector<Graph::vertex>(neighbours);
}

std::vector<Graph::vertex> Graph::bfs(vertex start) {
  //Perform a breadth first search on the entire graph
  std::vector<Graph::vertex> bfs;
  vertex current;
  std::queue<vertex> q;
  bfs.push_back(start);
  q.push(start);
  while (!q.empty()) {
    current = q.front();
    q.pop();

    for (auto neighbour: getNeighbours(current)) {
        if (std::find(bfs.begin(), bfs.end(),neighbour) == bfs.end()) {
          q.push(neighbour);  
          bfs.push_back(neighbour);       
        }
    }
  }
  return std::vector<Graph::vertex>(bfs);
}
