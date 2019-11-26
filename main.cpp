#include "Graph.h"
#include "ParserAirports.h"
#include "utils.h"

int main() {

    Graph grafoAeropuertos(true);
    // Dijkstra Graph
    /*
    grafoAeropuertos.agregarArista(1,2,2);
    grafoAeropuertos.agregarArista(1,3,4);
    grafoAeropuertos.agregarArista(2,3,1);
    grafoAeropuertos.agregarArista(2,4,7);
    grafoAeropuertos.agregarArista(3,5,3);
    grafoAeropuertos.agregarArista(5,4,2);
    grafoAeropuertos.agregarArista(5,4,2);
    grafoAeropuertos.agregarArista(5,6,5);
    grafoAeropuertos.agregarArista(4,6,1);
     */
    // another Disjkstra
    grafoAeropuertos.agregarArista(1,4,10);
    grafoAeropuertos.agregarArista(4,1,10);
    grafoAeropuertos.agregarArista(1,2,50);
    grafoAeropuertos.agregarArista(2,4,15);
    grafoAeropuertos.agregarArista(4,5,15);
    grafoAeropuertos.agregarArista(5,2,20);
    grafoAeropuertos.agregarArista(2,3,10);
    grafoAeropuertos.agregarArista(1,3,45);
    grafoAeropuertos.agregarArista(5,3,35);
    grafoAeropuertos.agregarArista(3,5,30);
    grafoAeropuertos.agregarArista(6,5,3);
    grafoAeropuertos.imprimir();
    //auto bfs_result = grafoAeropuertos.BFS();
    auto dfs_result = grafoAeropuertos.DFS();
    //printVector(bfs_result);
    printVector(dfs_result);
    // Dijkstra
    auto result = grafoAeropuertos.dijkstra(1);
    printDijkstra(result);
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
