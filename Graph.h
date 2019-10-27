#ifndef PROYECTO_AIRPORTS_GRAPH_H
#define PROYECTO_AIRPORTS_GRAPH_H

#include <iostream>
#include <map>

#include "Edge.h"
#include "Airport.h"

class Graph {
private:
    int vertices;
    int aristas;
    bool esDirigido = false;
    std::map <int, listaAdyacencia*> nodosGrafo;

public:
    explicit Graph () : vertices{0}, aristas{0} {}

    static nodoListaAdyacencia* crearNodoListaAdyacencia (int origen, int destino, std::map <int, Airport*> &aeropuertos) {
        auto* nuevoNodo = new nodoListaAdyacencia(origen, destino, aeropuertos);
        return nuevoNodo;
    }

    bool existeArista(int origen, int destino) {
        auto* actual = nodosGrafo[origen]->head;
        while (actual) {
            if (actual->idDestino == destino) {
                return true;
            }
            actual = actual->next;
        }
        return false;
    }

    void agregarArista(int origen, int destino, std::map <int, Airport*> &aeropuertos) {
        nodoListaAdyacencia* nuevoNodo = crearNodoListaAdyacencia(origen, destino, aeropuertos);
        if (!nodosGrafo[origen]) {
            nodosGrafo[origen] = new listaAdyacencia();
            nodosGrafo[origen]->head = nullptr;
            vertices++;
        }
        if (!existeArista(origen, destino) && origen != destino) {
            nuevoNodo->next = nodosGrafo[origen]->head;
            nodosGrafo[origen]->head = nuevoNodo;
            aristas++;
        }

        if (!esDirigido) {
            nuevoNodo = crearNodoListaAdyacencia(destino, origen, aeropuertos);
            if (!nodosGrafo[destino]) {
                nodosGrafo[destino] = new listaAdyacencia();
                nodosGrafo[destino]->head = nullptr;
                vertices++;
            }
            if (!existeArista(destino, origen) && destino != origen) {
                nuevoNodo->next = nodosGrafo[destino]->head;
                nodosGrafo[destino]->head = nuevoNodo;
            }
        }
    }

    void printGraph() {
        for (auto & it : nodosGrafo) {
            nodoListaAdyacencia* pCrawl;
            if (nodosGrafo[it.first]) {
                pCrawl = nodosGrafo[it.first]->head;
            } else {
                pCrawl = nullptr;
            }
            if (pCrawl) {
                std::cout << "\nAdjacency list of vertex " << it.first << "\n head ";
            }
            while (pCrawl) {
                std::cout << "-> " << pCrawl->idDestino << "(" << pCrawl->peso << ")";
                pCrawl = pCrawl->next;
            }
        }
        std::cout << std::endl << std::endl << "Vertices: " << vertices << "\nAristas: " << aristas << std::endl;
    }

};

#endif //PROYECTO_AIRPORTS_GRAPH_H
