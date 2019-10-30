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

    void killSelf() {
        if (this->next) {
            this->next->killSelf();
        }
        delete this;
    }

};

class listaAdyacencia {
public:
    nodoListaAdyacencia *head;
    ~listaAdyacencia() = default;
};

#endif //PROYECTO_AIRPORTS_EDGE_H
