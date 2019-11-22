#include "Graph.h"
#include "ParserAirports.h"

int main() {

    ParserAirports parserAirports("../demo.json");
    Graph grafoAeropuertos = parserAirports.generarGrafo();
    grafoAeropuertos.imprimir();
    Graph grafoPrim = grafoAeropuertos.algoritmoPrim();
    grafoPrim.imprimir();
    Graph grafoKruskal = grafoAeropuertos.algoritmoKruskal();
    grafoKruskal.imprimir();

    return 0;

}
