#ifndef PROYECTO_AIRPORTS_GRAPH_H
#define PROYECTO_AIRPORTS_GRAPH_H

#include <iostream>
#include <map>

#include "Edge.h"

class Graph {
private:
    int vertices;
    std::map <int, listaAdyacencia*> nodosGrafo;

public:
    explicit Graph (int vertices) {
        this->vertices = vertices;
    }

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
        }
        nuevoNodo->next = nodosGrafo[origen]->head;
        nodosGrafo[origen]->head = nuevoNodo;
        // Esta parte determina si es dirigido o no
        nuevoNodo = crearNodoListaAdyacencia(origen);
        if (!nodosGrafo[destino]) {
            nodosGrafo[destino] = new listaAdyacencia();
        }
        nuevoNodo->next = nodosGrafo[destino]->head;
        nodosGrafo[destino]->head = nuevoNodo;
    }

    void printGraph() {
        for (int v = 0; v < vertices; ++v) {
            nodoListaAdyacencia* pCrawl;
            if (nodosGrafo[v]) {
                pCrawl = nodosGrafo[v]->head;
            } else {
                pCrawl = nullptr;
            }
            if (pCrawl) {
                std::cout << "\nAdjacency list of vertex " << v << "\n head ";
            }
            while (pCrawl) {
                std::cout<<"-> "<<pCrawl->data;
                pCrawl = pCrawl->next;
            }
        }
    }
};

#endif //PROYECTO_AIRPORTS_GRAPH_H