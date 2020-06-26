#include <set>
#include <queue>
#include "graph.h"

void Graph::addVertex(vertex v) {
  //Insert a vertex with no edges
  weightedGraph_[v];
}

bool Graph::hasVertex(vertex v) {
  //Check if the vertex v, exists in the graph
  if (weightedGraph_.find(v) != weightedGraph_.end()){
    return true;
  }
  else{
    return false;
  }
}

void Graph::addEdge(vertex u, vertex v, weight w) {
  //Assumes that u & v have already been added to the graph
  //We need to record the same edge twice as this is an undirected graph
  weightedGraph_.at(u).insert({v, w}); //Inserting an edge between u and v, with weight w
  weightedGraph_.at(v).insert({u, w}); //Inserting an edge between v and u, with weight w
}


std::vector<Graph::vertex> Graph::getVertices(void) {
  //Iterate through the weightedGraph_ and push back each vertex to the vertices vector
  std::vector<Graph::vertex> vertices;
  for (auto v : weightedGraph_){
    vertices.push_back(v.first);
  }
  //return std::vector<Graph::vertex>();
  return vertices;
}

std::vector<Graph::vertex> Graph::bfs(vertex start) {
  //Perform a breadth first search on the entire graph
  std::vector<Graph::vertex> bfs;
  std::map<vertex, bool> visited;
  //no search target, no queue required

  //push first element into vector
  bfs.push_back(start);
  visited[start] = true;

  for (auto vert : weightedGraph_){
    for (auto neighbours : vert.second){
      //See if vertex has been previously checked
      bool checked = (visited.find(neighbours.first) != visited.end());
      if (checked == false){
        visited[neighbours.first] = true;   //add neighbour to map
        bfs.push_back(neighbours.first);
      }
    }
  }
  return bfs;
  //return std::vector<Graph::vertex>();
}
