#ifndef PROYECTO_AIRPORTS_GRAPH_H
#define PROYECTO_AIRPORTS_GRAPH_H

#include <iostream>
#include <map>

#include "Edge.h"

class Graph {
private:
    int vertices;
    int aristas;
    std::map <int, listaAdyacencia*> nodosGrafo;

public:
    explicit Graph () : vertices{0}, aristas{0} {}

    static nodoListaAdyacencia* crearNodoListaAdyacencia (int destino) {
        auto* nuevoNodo = new nodoListaAdyacencia;
        nuevoNodo->data = destino;
        nuevoNodo->next = nullptr;
        return nuevoNodo;
    }

    void addEdge(int origen, int destino) {
        nodoListaAdyacencia* nuevoNodo = crearNodoListaAdyacencia(destino);
        if (!nodosGrafo[origen]) {
            nodosGrafo[origen] = new listaAdyacencia();
            nodosGrafo[origen]->head = nullptr;
        }
        nuevoNodo->next = nodosGrafo[origen]->head;
        nodosGrafo[origen]->head = nuevoNodo;
        // Esta parte determina si es dirigido o no
        nuevoNodo = crearNodoListaAdyacencia(origen);
        if (!nodosGrafo[destino]) {
            nodosGrafo[destino] = new listaAdyacencia();
            nodosGrafo[destino]->head = nullptr;
        }
        nuevoNodo->next = nodosGrafo[destino]->head;
        nodosGrafo[destino]->head = nuevoNodo;
    }

    void printGraph() {
        for (auto it = nodosGrafo.begin(); it != nodosGrafo.end(); ++it) {
            nodoListaAdyacencia* pCrawl;
            if (nodosGrafo[it->first]) {
                pCrawl = nodosGrafo[it->first]->head;
            } else {
                pCrawl = nullptr;
            }
            if (pCrawl) {
                std::cout << "\nAdjacency list of vertex " << it->first << "\n head ";
            }
            while (pCrawl) {
                std::cout<<"-> "<<pCrawl->data;
                pCrawl = pCrawl->next;
            }
        }
    }
};

#endif //PROYECTO_AIRPORTS_GRAPH_H
