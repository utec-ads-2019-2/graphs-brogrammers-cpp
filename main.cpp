#include "Graph.h"
#include "ParserAirports.h"

int main() {

    Graph grafoAeropuertos;
    /*ParserAirports parserAirports("../demo.json");
    parserAirports.generarGrafo(grafoAeropuertos);
    grafoAeropuertos.printGraph();*/
    grafoAeropuertos.agregarArista(0, 1, 3.5);
    grafoAeropuertos.agregarArista(1, 4, 3.5);
    grafoAeropuertos.agregarArista(2, 3, 3.5);
    grafoAeropuertos.agregarArista(3, 4, 3.5);
    grafoAeropuertos.agregarVertice(5);
    grafoAeropuertos.printGraph();
    std::cout << std::boolalpha << grafoAeropuertos.esConexo() << std::endl;

    return 0;
}
