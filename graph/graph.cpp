#include "graph.h"

/// <summary>
/// ����� ������� � ����������� ����� � �������
/// </summary>
/// <param name="shortPath"></param>
/// <param name="visited"></param>
/// <returns></returns>
int minDistance(const std::vector<double>& shortPath, const std::vector<bool>& visited)
{
    double min = INF;
    int min_index = 0;

    for (std::size_t v = 0; v < shortPath.size(); ++v) {
        if (!visited[v] && shortPath[v] <= min) {
            min = shortPath[v];
            min_index = v;
        }
    }

    return min_index;
}

/// <summary>
/// ������� �������� ��� ������ � ���������������� ������
/// </summary>
/// <param name="graph"></param>
/// <param name="src"></param>
/// <param name="sink"></param>
/// <returns></returns>
Pair dijkstra(const Graph& graph, const int src, const int sink)
{
    std::vector<double> shortPath(graph.size(), INF);
    shortPath[src] = 0;

    std::vector<int> P(graph.size(), src);

    std::vector<bool> visited(graph.size(), false);

    int w;
    for (std::size_t i = 0; i < shortPath.size() - 1; ++i) {
        w = minDistance(shortPath, visited);
        visited[w] = true;
        for (int v = 0; v < shortPath.size(); ++v) {
            if (!visited[v] && graph[w][v] != INF && shortPath[v] > (shortPath[w] + graph[w][v])) {
                shortPath[v] = shortPath[w] + graph[w][v];
                P[v] = w;
            }
        }
    }

    // ����� ���� �� ��������� � �������� �������.
    std::vector<int> path;
    for (int node = sink; node != src;) {
        path.push_back(node);
        node = P[node];
    }
    path.push_back(src);
    std::reverse(path.begin(), path.end());

    if (shortPath[sink] == INF) {
        return Pair(int(INF), { 0 });
    }
    return Pair(int(shortPath[sink]), path);
}

/// <summary>
/// ������� ����� �� �����
/// </summary>
/// <param name="graph"></param>
/// <param name="node1"></param>
/// <param name="node2"></param>
void removeEdge(Graph& graph, Mode graphType, const int node1, const int node2)
{
    if (graphType == oriented) {
        graph[node1][node2] = graph[node2][node1] = INF;
    }
    else {
        graph[node1][node2] = INF;
    }
}

/// <summary>
/// ������� ������� �� �����
/// </summary>
/// <param name="graph"></param>
/// <param name="node"></param>
void removeNode(Graph& graph, const int node)
{
    for (std::size_t i = 0; i < graph.size(); ++i) {
        graph[node][i] = graph[i][node] = INF;
    }
}

/// <summary>
/// ����� ����� ���� ��� ������ �� �������
/// </summary>
/// <param name="graph"></param>
/// <param name="begin"></param>
/// <param name="end"></param>
/// <returns></returns>
Pair lengthPath(const Graph& graph, const iteratorVectInt begin, const iteratorVectInt end)
{
    int length = 0;
    int first;
    int last;
    std::vector<int> path(begin, end);
    for (auto node = path.begin() + 1; node != path.end(); ++node) {
        first = *(node - 1);
        last = *node;
        length += graph[first][last];
    }

    return Pair(length, path);
}

/// <summary>
/// ��������� ������� ���� � �������
/// </summary>
/// <param name="what"></param>
/// <param name="where"></param>
/// <returns></returns>
bool presencePath(const Map& what, const MultiMap& where)
{
    auto equalLengths = where.equal_range(what.begin()->first);
    for (auto& itWhere = equalLengths.first; itWhere != equalLengths.second; ++itWhere) {
        if (what.begin()->second == itWhere->second) {
            return false;
        }
    }
    return true;
}

/// <summary>
/// �������� ����
/// </summary>
/// <param name="graph"></param>
/// <param name="src"></param>
/// <param name="sink"></param>
/// <param name="K"></param>
/// <returns></returns>
MultiMap yen(const Graph& graph, Mode graphType, const int src, const int sink, const int K)
{
    Graph tempGraph;

    MultiMap paths;             // ������ ��������� �����.
    paths.insert(dijkstra(graph, src, sink));
    MultiMap::iterator itPaths = paths.begin();
    MultiMap candidates;        // ������ ����������.

    Map rootPath;               // �������� ���� - ��������� ����� ����������� ����.
    Map::iterator itRootPath;
    Map spurPath;               // ���� ���������.
    Map::iterator itSpurPath;
    Map totalPath;              // ����� ����(totalPath) = rootPath + spurPath.

    int spurNode;               // ������� ���������.

    // ��������������� ����������.
    int length;
    std::vector<int> nodes;
    for (int k = 1; k < K; ++k, ++itPaths) {
        for (int i = 0; i < itPaths->second.size() - 1; ++i) {
            tempGraph = graph;

            spurNode = itPaths->second[i];

            rootPath.clear();
            rootPath.insert(lengthPath(tempGraph, itPaths->second.begin(), itPaths->second.begin() + i + 1));
            itRootPath = rootPath.begin();

            // �������� ����� �� �����
            for (auto& p : paths) {
                if (i >= p.second.size()) continue;
                std::vector<int> path(p.second.begin(), p.second.begin() + i + 1);
                if (itRootPath->second == path) {
                    removeEdge(tempGraph, graphType, p.second[i], p.second[i + 1]);
                }
            }

            // �������� ���� ������, ������������ � �������� ���� ����� ������� ���������
            for (auto rootPathNode : itRootPath->second) {
                if (rootPathNode != spurNode) {
                    removeNode(tempGraph, rootPathNode);
                }
            }

            spurPath.clear();
            spurPath.insert(dijkstra(tempGraph, spurNode, sink));
            itSpurPath = spurPath.begin();
            // ���� ���� ��� (����� ������������), �� ��������� �� ��������� ��������.
            if (itSpurPath->second.size() == 1) continue;

            // ��������� ��� ��������� ���� � ���� ���������
            int totalLength = itRootPath->first + itSpurPath->first;
            //����������� ��������� ���� � ���� ���������
            itRootPath->second.insert(itRootPath->second.end(), itSpurPath->second.begin() + 1, itSpurPath->second.end());
            totalPath.clear();
            totalPath.emplace(totalLength, itRootPath->second);

            if (presencePath(totalPath, candidates) && presencePath(totalPath, paths)) {
                candidates.insert(totalPath.begin(), totalPath.end());
            }
        }
        if (candidates.empty()) break;

        length = candidates.begin()->first;
        nodes = candidates.begin()->second;
        paths.insert(Pair(length, nodes));
        candidates.erase(candidates.begin());
    }

    return paths;
}

/// <summary>
/// ���������� ����
/// </summary>
/// <param name="graph"></param>
void showGraph(const Graph& graph)
{
    for (std::size_t i = 0; i < graph.size(); ++i) {
        for (std::size_t j = 0; j < graph.size(); ++j) {
            std::cout << std::setw(4) << std::right << graph[i][j];
        }
        std::cout << std::endl;
    }
}

/// <summary>
/// ���������� ��������� ����
/// </summary>
/// <param name="paths"></param>
void showPaths(const MultiMap& paths)
{
    int counter = 0;
    for (auto& i : paths) {
        std::cout.precision(0);
        std::cout << std::setw(5) << std::right << ++counter
            << ") Length: " << i.first << "\tPath: ";
        for (auto j : i.second) {
            std::cout << j << " ";
        }
        std::cout << std::endl;
    }
}
