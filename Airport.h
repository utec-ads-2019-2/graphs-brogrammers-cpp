#ifndef GRAPHS_BROGRAMMERS_CPP_AIRPORT_H
#define GRAPHS_BROGRAMMERS_CPP_AIRPORT_H

#include <utility>
#include <vector>

using namespace std;
class Airport{
    vector<int> destinations;
    std::string city;
    std::string name;
    std::string country;
    long double longitude;
    long double latitude;

    explicit Airport(vector<int> _dest) : destinations{std::move(_dest)} {};

    explicit Airport(vector<int> _dest,
                     long double _longitude,
                     long double _latitude
    ) : destinations{std::move(_dest)},
        longitude{_longitude},
        latitude{_latitude} {};

    friend class ParserAirports;
};

#endif //GRAPHS_BROGRAMMERS_CPP_AIRPORT_H
