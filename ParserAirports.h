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
            destinations.reserve(destinos_vec.size());
            for (const auto& destino : destinos_vec) {
                destinations.push_back(stoi(destino));      // add destinations to vector<int>
            }
            Airport* airport_obj = new Airport(destinations);    //create pointer to object Airport
            //vec_airports.push_back(airport_obj);
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
