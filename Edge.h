#ifndef PROYECTO_AIRPORTS_EDGE_H
#define PROYECTO_AIRPORTS_EDGE_H

#include <cmath>

#include "Airport.h"

class nodoListaAdyacencia {
public:
    int idDestino = 0;
    double peso = 0;
    nodoListaAdyacencia* next = nullptr;

    nodoListaAdyacencia() = default;
    nodoListaAdyacencia(int destino, double peso) : idDestino{destino}, peso{peso} {}
    ~nodoListaAdyacencia() = default;
};

class listaAdyacencia {
public:
    nodoListaAdyacencia *head;
    ~listaAdyacencia() = default;
};

class nodoMinHeap {
public:
    int valor = 0;
    double clave = 0;
    nodoMinHeap() = default;
    nodoMinHeap(int value, double key) : valor{value}, clave{key} {}
};

class minHeap {
public:
    int numeroNodos;
    int capacidad;
    std::map <int, int> posicion{};
    std::map <int, nodoMinHeap*> array{};
    minHeap(int nodos, int capacity) : numeroNodos{nodos}, capacidad{capacity} {}
    ~minHeap() {
        for (auto element : array) {
            delete element.second;
        }
    }
};

#endif //PROYECTO_AIRPORTS_EDGE_H
