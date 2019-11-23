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

#endif //GRAPHS_BROGRAMMERS_CPP_UTILS_H
