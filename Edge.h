#ifndef PROYECTO_AIRPORTS_EDGE_H
#define PROYECTO_AIRPORTS_EDGE_H

#include <cmath>

#include "Airport.h"

class nodoListaAdyacencia {
public:
    int idDestino = 0;
    double peso = 0;
    nodoListaAdyacencia* next = nullptr;

    nodoListaAdyacencia(int origen, int destino, std::map <int, Airport*> &data) {
        double radioTierra = 6371;
        Airport *airportOrigen = data[origen];
        Airport *airportDestino = data[destino];
        double distanciaLatitud = gradosARadianes(airportDestino->latitud - airportOrigen->latitud);
        double distanciaLongitud = gradosARadianes(airportDestino->longitud - airportOrigen->longitud);
        double a = sin(distanciaLatitud / 2) * sin(distanciaLatitud / 2) + cos(gradosARadianes(airportOrigen->latitud)) * cos(gradosARadianes(airportDestino->latitud)) * sin(distanciaLongitud / 2) * sin(distanciaLongitud / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        double distancia = radioTierra * c;
        peso = distancia;
        idDestino = destino;
    }

    static double gradosARadianes(double grados) {
        return grados * (M_PI / 180);
    }

};

class listaAdyacencia {
public:
    nodoListaAdyacencia *head;
};

#endif //PROYECTO_AIRPORTS_EDGE_H
