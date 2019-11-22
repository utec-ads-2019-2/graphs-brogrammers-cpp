#include "Graph.h"
#include "ParserAirports.h"

void imprimirMatriz(std::vector <std::vector <double>> matrizFloyd) {
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

int main() {

    Graph grafoAeropuertos(true);
    grafoAeropuertos.agregarArista(0,3,10);
    grafoAeropuertos.agregarArista(0,1,5);
    grafoAeropuertos.agregarArista(1,2,3);
    grafoAeropuertos.agregarArista(2,3,1);
    grafoAeropuertos.imprimir();
    std::vector <std::vector <double>> matrizFloyd = grafoAeropuertos.algoritmoFloydWarshal();
    imprimirMatriz(matrizFloyd);

    return 0;

}
