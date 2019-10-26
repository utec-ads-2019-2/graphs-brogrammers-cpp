#ifndef GRAPHS_BROGRAMMERS_CPP_AIRPORT_H
#define GRAPHS_BROGRAMMERS_CPP_AIRPORT_H

#include <utility>
#include <vector>

using namespace std;
class Airport{
    int id;
    vector<int> destinations;

    explicit Airport(int _id, vector<int> _dest) : id{_id}, destinations{std::move(_dest)} {};

    friend class ParserAirports;
};

#endif //GRAPHS_BROGRAMMERS_CPP_AIRPORT_H
