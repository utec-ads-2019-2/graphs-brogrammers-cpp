#include "Graph.h"
#include "ParserAirports.h"

int main() {

    Graph grafoAeropuertos;
    /*ParserAirports parserAirports("../demo.json");
    parserAirports.generarGrafo(grafoAeropuertos);
    grafoAeropuertos.printGraph();*/
    grafoAeropuertos.agregarArista(0, 1, 1);
    grafoAeropuertos.agregarArista(0, 2, 6);
    grafoAeropuertos.agregarArista(1, 2, 4);
    grafoAeropuertos.agregarArista(1, 3, 3);
    grafoAeropuertos.agregarArista(2, 3, 1);
    grafoAeropuertos.agregarArista(1, 4, 1);
    grafoAeropuertos.agregarArista(3, 4, 1);
    grafoAeropuertos.printGraph();
    grafoAeropuertos.algoritmoKruskal();


    return 0;
}
