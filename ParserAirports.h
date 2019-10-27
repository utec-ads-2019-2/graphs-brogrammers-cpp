#ifndef GRAPHS_BROGRAMMERS_CPP_PARSERAIRPORTS_H
#define GRAPHS_BROGRAMMERS_CPP_PARSERAIRPORTS_H

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "nlohmann/json.hpp"
#include "Airport.h"
#include <map>

using namespace std;
using json = nlohmann::json;

class ParserAirports{
public:
    //vector<Airport*> vec_airports;
    map<int, Airport*> map_airports;
    string json_file;

    void printValues(){
        for(auto it = map_airports.begin(); it != map_airports.end(); ++it) {
            cout << "Id Airport: " << it->first << " \n";
            cout << "Destinations: " << '\n';
            for(auto element : (it->second)->destinations){
                cout << element << " ";
            }
            cout << '\n';
            cout << "Longitude: " << (std::fixed) << std::setprecision(6) << (it->second)->longitude << '\n';
            cout << "Latitude: " << (std::fixed) << std::setprecision(6) << (it->second)->latitude << '\n';
            cout << '\n';
        }
    }

    void readFile(){
        cout << "Read file: " << json_file << '\n';
        std::ifstream i(json_file);
        json j;
        i >> j;
        for (json::iterator it = j.begin(); it != j.end(); ++it) {  // iterate over each element Airport
            json object = *it;
            //Get Id Airport
            string id = object.at("Id");
            int id_airport = stoi(id);
            // Get Destinations
            vector<int> destinations;
            std::vector<std::string> destinos_vec = object.at("destinations");
            //Get Latitude and Longitude
            string lati = object.at("Latitude");
            string longi = object.at("Longitude");
            //cout << "str longi: " << longi << '\n';
            double latitude = stod(lati);
            double longitude = stod(longi);
            //cout << "d longi: " << std::fixed << std::setprecision(6) << longitude << '\n';
            destinations.reserve(destinos_vec.size());
            for (const auto& destino : destinos_vec) {
                destinations.push_back(stoi(destino));      // agrega destinos to vector<int> destinations
            }
            //Airport* airport_obj = new Airport(destinations);    //create pointer to object Airport
            Airport* airport_obj = new Airport(destinations, longitude, latitude);    //create pointer to object Airport
            map_airports[id_airport] = airport_obj;
        }
    }

public:
    ParserAirports()= default;;
    ParserAirports(string _json_file) : json_file(_json_file){
        readFile();
    }

    ~ParserAirports(){
        cout << "Destructor called \n";
        for(auto it = map_airports.begin(); it != map_airports.end(); ++it){
            delete it->second;
        }
    }

    void print(){
        printValues();
    }

};


#endif //GRAPHS_BROGRAMMERS_CPP_PARSERAIRPORTS_H
