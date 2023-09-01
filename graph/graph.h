#ifndef GRAPH_H
#define GRAPH_H

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <limits>
#include <map>
#include <vector>

constexpr double INF = std::numeric_limits<double>::infinity();

enum Mode { oriented , undirected };

typedef std::vector<std::vector<double>> Graph;
typedef std::multimap<int, std::vector<int>> MultiMap;
typedef std::map<int, std::vector<int>> Map;
typedef std::pair<int, std::vector<int>> Pair;
typedef std::vector<int>::iterator iteratorVectInt;

int minDistance(const std::vector<double>& shortPath, const std::vector<bool>& visited);
Pair dijkstra(const Graph& graph, const int src, const int sink);

void removeEdge(Graph& graph, Mode graphType, const int node1, const int node2);
void removeNode(Graph& graph, const int node);
Pair lengthPath(const Graph& graph, const iteratorVectInt begin, const iteratorVectInt end);
bool presencePath(const Map& what, const MultiMap& where);
MultiMap yen(const Graph& graph, Mode graphType, const int src, const int sink, const int K);

void showGraph(const Graph& graph);
void showPaths(const MultiMap& paths);

#endif // !GRAPH_H
