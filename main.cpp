#include "Graph.h"
#include "ParserAirports.h"

int main() {

    ParserAirports parserAirports("demo.json");
    Graph grafoAeropuertos = parserAirports.generarGrafo();
    grafoAeropuertos.printGraph();

    return 0;
}
