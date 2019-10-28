#include "Graph.h"
#include "ParserAirports.h"

int main() {

    Graph grafoAeropuertos;
    /*ParserAirports parserAirports("../demo.json");
    parserAirports.generarGrafo(grafoAeropuertos);
    grafoAeropuertos.printGraph();*/
    grafoAeropuertos.agregarArista(0, 2, 1.1);
    grafoAeropuertos.agregarArista(0, 4, 6.6);
    grafoAeropuertos.agregarArista(2, 4, 4.4);
    grafoAeropuertos.agregarArista(2, 6, 3.3);
    grafoAeropuertos.agregarArista(4, 6, 1.1);
    grafoAeropuertos.agregarArista(2, 8, 1.1);
    grafoAeropuertos.agregarArista(6, 8, 1.1);
    grafoAeropuertos.printGraph();
    grafoAeropuertos.algoritmoPrim();
    //Test AngeLinux
    /*std::cout << std::boolalpha << grafoAeropuertos.esConexo() << std::endl;
    grafoAeropuertos.agregarArista(5, 6, 3);
    grafoAeropuertos.agregarArista(6, 7, 2);
    std::cout << std::boolalpha << grafoAeropuertos.esConexo() << std::endl;
    std::cout << std::boolalpha << grafoAeropuertos.esBipartito() << std::endl;*/
    return 0;
}
