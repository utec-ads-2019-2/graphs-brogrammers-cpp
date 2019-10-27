#ifndef PROYECTO_AIRPORTS_PARSERAIRPORTS_H
#define PROYECTO_AIRPORTS_PARSERAIRPORTS_H

#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include <utility>
#include "nlohmann/json.hpp"

#include "Graph.h"
#include "Airport.h"

using json = nlohmann::json;

class ParserAirports{
public:
    std::map <int, Airport*> aeropuertos;
    std::string json_file;

    static std::vector <int> extraerDestinos(json &objeto) {
        std::vector <int> destinos;
        std::vector <std::string> destinos_vec = objeto.at("destinations");
        destinos.reserve(destinos_vec.size());
        for (const auto& destino : destinos_vec) {
            destinos.push_back(std::stoi(destino));
        }
        return destinos;
    }

    void crearAristas(Graph &grafoAeropuertos) {
        grafoAeropuertos.cargarData(aeropuertos);
        for (auto element : aeropuertos) {
            int idOrigen = element.first;
            std::vector <int> destinos = element.second->destinos;
            for (auto dest : destinos) {
                grafoAeropuertos.agregarAristaAeropuerto(idOrigen, dest);
            }
        }
    }

    void readFile(){
        std::ifstream i(json_file);
        json j;
        i >> j;
        for (auto & it : j) {
            json objeto = it;
            std::string id = objeto.at("Id");
            int id_aeropuerto = std::stoi(id);

            std::string ciudad = objeto.at("City");
            std::string nombre = objeto.at("Name");
            std::string pais   = objeto.at("Country");
            std::string lon    = objeto.at("Longitude");
            double longitud    = std::stod(lon);
            std::string lat    = objeto.at("Latitude");
            double latitud     = std::stod(lat);
            std::vector <int> destinos = extraerDestinos(it);

            auto* aeropuerto = new Airport(ciudad, nombre, pais, longitud, latitud, destinos);

            aeropuertos [id_aeropuerto] = aeropuerto;
        }
    }

    ParserAirports()= default;;

    explicit ParserAirports(std::string _json_file) : json_file(std::move(_json_file)){
        readFile();
    }

    void generarGrafo(Graph &grafoAeropuertos) {
        crearAristas(grafoAeropuertos);
    }

    void print(){
        for (auto element : aeropuertos) {
            std::cout << "id Airport: " << element.first << '\n';
            std::cout << "Destinations: " << '\n';
            for(auto id_destinos : element.second->destinos){
                std::cout << id_destinos << " ";
            }
            std::cout << '\n';
        }
    }

    ~ParserAirports(){
        std::cout << "Destructor parser" << std::endl;
        for(auto element : aeropuertos){
            delete element.second;
        }
    }

};

#endif //PROYECTO_AIRPORTS_PARSERAIRPORTS_H
