#include "Graph.h"

int main() {
    Graph gh(5);
    gh.addEdge(0, 1);
    gh.addEdge(1, 2);
    gh.addEdge(2, 3);

    gh.printGraph();

    return 0;
}