#include "Graph.h"
#include "ParserAirports.h"

int main() {
    /*Graph gh;
    gh.addEdge(0, 1);
    gh.addEdge(1, 2);
    gh.addEdge(2, 3);

    gh.printGraph();*/

    ParserAirports parser("../demo.json");
    parser.print();

    return 0;
}