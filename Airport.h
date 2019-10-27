#ifndef PROYECTO_AIRPORTS_AIRPORT_H
#define PROYECTO_AIRPORTS_AIRPORT_H

#include <string>
#include <utility>
#include <vector>

using namespace std;

class Airport {
public:
    std::string ciudad;
    std::string nombre;
    std::string pais;
    double longitud;
    double latitud;
    std::vector <int> destinos;

    Airport(std::string ciudad,
            std::string nombre,
            std::string pais,
            double longitud,
            double latitud,
            std::vector <int> destinos
    ) : ciudad{std::move(ciudad)},
        nombre{std::move(nombre)},
        pais{std::move(pais)},
        longitud{longitud},
        latitud{latitud},
        destinos{std::move(destinos)} {}

};

#endif //PROYECTO_AIRPORTS_AIRPORT_H
