#include "graph/graph.h"

int main()
{
    setlocale(LC_ALL, "ru-RU");
    // ����������������� ����
    Graph undirectedGraph = { { 0,   2,   6,   8,   INF, INF, 3,   INF, INF },
                              { 2,   0,   9,   3,   INF, 4,   9,   INF, INF },
                              { 6,   9,   0,   7,   INF, INF, INF, INF, INF },
                              { 8,   3,   7,   0,   5,   5,   INF, INF, INF },
                              { INF, INF, INF, 5,   0,   INF, 8,   9,   INF },
                              { INF, 4,   INF, 5,   INF, 0,   INF, 6,   4   },
                              { 3,   9,   INF, INF, 8,   INF, 0,   INF, INF },
                              { INF, INF, INF, INF, 9,   6,   INF, 0,   1   },
                              { INF, INF, INF, INF, INF, 4,   INF, 1,   0   },
                            };
    // ��������������� ����
    Graph orientedGraph = { { 0,   3,   INF, INF, 2,   7   },
                            { INF, 0,   5,   INF, 1,   INF },
                            { INF, INF, 0,   2,   INF, INF },
                            { INF, 4,   INF, 0,   INF, INF },
                            { INF, INF, INF, 9,   0,   4   },
                            { 8,   INF, INF, 3,   INF, 0   },
                          };
    
    int src;
    std::cout << "������� ��������: ";
    std::cin >> src;

    int sink;
    std::cout << "������� �������� �������: ";
    std::cin >> sink;

    int K;
    std::cout << "������� ���������� �����, ������� ����� �����: ";
    std::cin >> K;

    MultiMap paths = yen(undirectedGraph, undirected, src, sink, K);
    
    std::cout << "\n����:" << std::endl;
    showGraph(orientedGraph);
    std::cout << "\n��������� ����:" << std::endl;
    showPaths(paths);

    return 0;
}
