#include "Graph.h"
#include "ParserAirports.h"

int main() {

    Graph grafoAeropuertos;
    ParserAirports parserAirports("../demo.json");
    parserAirports.generarGrafo(grafoAeropuertos);
    grafoAeropuertos.imprimir();

    return 0;

}
