#include "Graph.h"
#include "ParserAirports.h"

void imprimirMatriz(std::vector <std::vector <double>> &matrizFloyd) {
    for (unsigned int i = 0; i < matrizFloyd.size(); ++i) {
        for (unsigned int j = 0; j < matrizFloyd.size(); ++j) {
            if (matrizFloyd[i][j] == INT_MAX) {
                std::cout << std::setw(4) << "INF";
            } else {
                std::cout << std::setw(4) << matrizFloyd[i][j];
            }
        }
        std::cout << std::endl;
    }
}

void imprimirVector(std::vector <std::pair <int, double>> &resultado) {
    std::cout << "Vertice\tDistancia desde punto origen" << std::endl;
    for (auto &it : resultado) {
        std::cout << it.first << '\t';
        if (it.second == INT_MAX) {
            std::cout << "INF" << std::endl;
        } else {
            std::cout << it.second << std::endl;
        }
    }
}

int main() {

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

    return 0;

}
