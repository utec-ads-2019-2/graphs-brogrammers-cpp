#ifndef PROYECTO_AIRPORTS_PARSERAIRPORTS_H
#define PROYECTO_AIRPORTS_PARSERAIRPORTS_H

#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include "nlohmann/json.hpp"

#include "Airport.h"

using json = nlohmann::json;

class ParserAirports{
public:
    std::map <int, Airport*> aeropuertos;
    std::string json_file;

    void printValues(){
        for (auto element : aeropuertos) {
            std::cout << "id Airport: " << element.first << '\n';
            std::cout << "Destinations: " << '\n';
            for(auto id_destinos : element.second->destinations){
                std::cout << id_destinos << " ";
            }
            std::cout << '\n';
        }
    }

    std::vector <int> extraerDestinos(json &objeto) {
        std::vector <int> destinos;
        std::vector <std::string> destinos_vec = objeto.at("destinations");
        destinos.reserve(destinos_vec.size());
        for (const auto& destino : destinos_vec) {
            destinos.push_back(std::stoi(destino));
        }
        return destinos;
    }

    void readFile(){
        std::ifstream i(json_file);
        json j;
        i >> j;
        for (json::iterator it = j.begin(); it != j.end(); ++it) {
            json objeto = *it;
            std::string id = objeto.at("Id");
            int id_aeropuerto = std::stoi(id);

            std::string ciudad = objeto.at("City");
            std::string nombre = objeto.at("Name");
            std::string pais   = objeto.at("Country");
            std::string lon    = objeto.at("Longitude");
            double longitud    = std::stod(lon);
            std::string lat    = objeto.at("Latitude");
            double latitud     = std::stod(lat);
            std::vector <int> destinos = extraerDestinos(*it);
            
            Airport* aeropuerto = new Airport(ciudad, nombre, pais, longitud, latitud, destinos);

            aeropuertos [id_aeropuerto] = aeropuerto;
        }
    }

    ParserAirports()= default;;
    ParserAirports(string _json_file) : json_file(_json_file){
        readFile();
    }

    ~ParserAirports(){
        cout << "Destructor called \n";
        for(auto element : aeropuertos){
            delete element.second;
        }
    }

    void print(){
        printValues();
    }

};

#endif //PROYECTO_AIRPORTS_PARSERAIRPORTS_H
