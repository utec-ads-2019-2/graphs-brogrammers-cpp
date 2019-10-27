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

    nodoListaAdyacencia(int origen, int destino, double peso) : idDestino{destino}, peso{peso} {}

};

class listaAdyacencia {
public:
    nodoListaAdyacencia *head;
};

#endif //PROYECTO_AIRPORTS_EDGE_H
