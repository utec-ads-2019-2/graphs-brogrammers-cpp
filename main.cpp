#include "Graph.h"
#include "ParserAirports.h"
#include "utils.h"

int main() {

    ParserAirports parserAirports("../airports.json");
    Graph graph = parserAirports.generarGrafo();
    //graph.imprimir();
    Graph grafoAA = graph.algoritmoAAsterisco(3782, 502);
    grafoAA.imprimir();
    return 0;

}
