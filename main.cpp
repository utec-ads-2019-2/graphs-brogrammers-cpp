#include "Graph.h"
#include "ParserAirports.h"
#include "utils.h"

int main() {

    Graph grafoAeropuertos(true);

    grafoAeropuertos.agregarArista(0,1,1);
    grafoAeropuertos.agregarArista(0,2,1);
    grafoAeropuertos.agregarArista(1,2,1);
    grafoAeropuertos.agregarArista(2,0,1);
    grafoAeropuertos.agregarArista(2,3,1);


    grafoAeropuertos.agregarArista(0,4,1);
    grafoAeropuertos.agregarArista(1,2,1);
    grafoAeropuertos.agregarArista(1,3,1);
    grafoAeropuertos.agregarArista(1,4,1);
    grafoAeropuertos.agregarArista(2,3,1);
    grafoAeropuertos.agregarArista(3,4,1);

    grafoAeropuertos.imprimir();
    auto bfs_result = grafoAeropuertos.BFS();
    printVector(bfs_result);
    //std::vector <std::vector <double>> matrizFloyd = grafoAeropuertos.algoritmoFloydWarshal();
    //imprimirMatriz(matrizFloyd);


    /*
    Graph grafo(true);
    grafo.agregarArista(0,1,-1);
    grafo.agregarArista(0,2,4);
    grafo.agregarArista(1,2,3);
    grafo.agregarArista(1,3,2);
    grafo.agregarArista(1,4,2);
    grafo.agregarArista(3,2,5);
    grafo.agregarArista(3,1,1);
    grafo.agregarArista(4,3,-3);

    grafo.imprimir();

    std::vector <std::pair <int, double>> bellmanFord = grafo.algoritmoBellmanFord(1);
    imprimirVector(bellmanFord);
     */
    return 0;

}
