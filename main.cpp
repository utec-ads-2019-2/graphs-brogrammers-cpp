#include "Graph.h"
#include "ParserAirports.h"
#include "utils.h"

int main() {

    ParserAirports parserAirports("../demo.json");
    Graph graph = parserAirports.generarGrafo();
    graph.imprimir();
    return 0;

}
