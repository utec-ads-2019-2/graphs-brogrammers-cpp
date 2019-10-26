#ifndef GRAPHS_BROGRAMMERS_CPP_PARSERAIRPORTS_H
#define GRAPHS_BROGRAMMERS_CPP_PARSERAIRPORTS_H

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "nlohmann/json.hpp"
#include "Airport.h"

using namespace std;
using json = nlohmann::json;

class ParserAirports{
private:
    vector<Airport*> vec_airports;
    string json_file;

    void printValues(){
        for(auto element : vec_airports){   // each airport
            cout << "id Airport: " << element->id << '\n';
            cout << "Destinations: " << '\n';
            for(auto id_destinos : element->destinations){
                cout << id_destinos << " ";
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
            //cout << "Print destinos: " << '\n';
            //for(auto element : destinations) cout << element << " ";
            //cout << '\n';
            Airport* airport_obj = new Airport(id_airport, destinations);    //create pointer to object Airport
            vec_airports.push_back(airport_obj);
        }
    }

public:
    ParserAirports()= default;;
    ParserAirports(string _json_file) : json_file(_json_file){
        readFile();
    }

    ~ParserAirports(){
        cout << "Destructor called \n";
        for(auto element : vec_airports){
            delete element;
        }
    }

    void print(){
        printValues();
    }

};


#endif //GRAPHS_BROGRAMMERS_CPP_PARSERAIRPORTS_H
