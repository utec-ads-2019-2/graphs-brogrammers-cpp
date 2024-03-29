#ifndef GRAPHS_BROGRAMMERS_CPP_UTILS_H
#define GRAPHS_BROGRAMMERS_CPP_UTILS_H
#include<vector>
#include<iostream>

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

void printVector(std::vector<int>& result){
    for(auto const& element : result){
        std::cout << element << " ";
    }
    std::cout << '\n';
    std::cout << '\n';
}

void printDijkstra(std::map<int, int>& dist){
    printf("Vertex \t Distance from Source\n");
    for(auto const& element : dist){
        if(dist[element.first] == INT_MAX){
            printf("%d \t\t %s\n", element.first, "INF");
        }
        else{
            printf("%d \t\t %d\n", element.first, dist[element.first]);
        }
    }
}

static double gradosARadianes(double grados) {
    return grados * (M_PI / 180);
}

double obtenerPeso(int origen, int destino, std::map <int, Airport*>& datos) {
    double radioTierra = 6371;
    Airport *airportOrigen = datos.find(origen)->second;
    Airport *airportDestino = datos.find(destino)->second;
    double distanciaLatitud = gradosARadianes(airportDestino->latitud - airportOrigen->latitud);
    double distanciaLongitud = gradosARadianes(airportDestino->longitud - airportOrigen->longitud);
    double a = sin(distanciaLatitud / 2) * sin(distanciaLatitud / 2) + cos(gradosARadianes(airportOrigen->latitud)) * cos(gradosARadianes(airportDestino->latitud)) * sin(distanciaLongitud / 2) * sin(distanciaLongitud / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distancia = radioTierra * c;
    return distancia;
}

#endif //GRAPHS_BROGRAMMERS_CPP_UTILS_H
