#include "Graph.h"
#include "ParserAirports.h"

int main() {

    Graph grafoAeropuertos;
    ParserAirports parserAirports("../airports.json");
    parserAirports.generarGrafo(grafoAeropuertos);
    //grafoAeropuertos.printGraph();
    /*Graph grafoPrim;
    grafoAeropuertos.algoritmoPrim(grafoPrim);
    grafoPrim.printGraph();*/
    Graph grafoKruskal;
    grafoAeropuertos.algoritmoKruskal(grafoKruskal);
    grafoKruskal.printGraph();
    /*grafoAeropuertos.agregarArista(0, 1, 1.1);
    grafoAeropuertos.agregarArista(0, 2, 6.6);
    grafoAeropuertos.agregarArista(1, 2, 4.4);
    grafoAeropuertos.agregarArista(1, 3, 3.3);
    grafoAeropuertos.agregarArista(2, 3, 1.1);
    grafoAeropuertos.agregarArista(1, 4, 1.1);
    grafoAeropuertos.agregarArista(3, 4, 1.1);
    grafoAeropuertos.printGraph();
    grafoAeropuertos.algoritmoPrim();*/
    //Test AngeLinux
    /*std::cout << std::boolalpha << grafoAeropuertos.esConexo() << std::endl;
    grafoAeropuertos.agregarArista(5, 6, 3);
    grafoAeropuertos.agregarArista(6, 7, 2);
    std::cout << std::boolalpha << grafoAeropuertos.esConexo() << std::endl;
    std::cout << std::boolalpha << grafoAeropuertos.esBipartito() << std::endl;*/
    return 0;
}
