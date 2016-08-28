#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>
#include "Astar.hpp"

int main(int argc, const char * argv[]) {
    freopen("demo.txt", "r", stdin);
    
    int N;
    std::cin >> N;
    
    std::vector<std::string> nodes;
    std::string node;
    for (int n = 0; n < N; n++) {
        std::cin >> node;
        nodes.emplace_back(node);
    }
    
    std::map<std::string, int> nodes_label;
    int label = 0;
    for (auto &node : nodes) {
        nodes_label[node] = label++;
    }
    
    int ** graph = new int*[N];
    for (int src = 0; src < N; src++) {
        graph[src] = new int[N];
        for (int dest = 0; dest < N; dest++) {
            if (src == dest) {
                graph[src][dest] = 0;
            } else {
                graph[src][dest] = std::numeric_limits<int>::max();
            }
        }
    }

    int connections;
    std::cin >> connections;
    std::string src, dest;
    int weight;
    for (int i = 0; i < connections; i++) {
        std::cin >> src >> dest >> weight;
        graph[nodes_label[src]][nodes_label[dest]] = weight;
        graph[nodes_label[dest]][nodes_label[src]] = weight;
    }
    
    std::string initial, goal;
    std::cin >> initial >> goal;
    
    std::map<int, int> estimated;
    std::string from;
    int h, value;
    std::cin >> h;
    for (int e = 0; e < h; e++) {
        std::cin >> from >> value;
        estimated[nodes_label[from]] = value;
    }
    
    std::vector<size_t> trace = Astar<int>(N, (const int **)graph, nodes_label[initial], nodes_label[goal]);
    
    std::cout << initial << " -> " << goal << ":\n";
    std::for_each(trace.begin(), trace.end(), [&nodes](const size_t node) {
        std::cout << nodes.at(node) << ' ';
    });
    std::cout << '\n';
    
    for (int i = 0; i < N; i++) {
        delete [] graph[i];
    }
    delete [] graph;
    return 0;
}
