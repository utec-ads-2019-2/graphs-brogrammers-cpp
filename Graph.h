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
    bool esDirigido = true;
    std::map <int, listaAdyacencia*> nodosGrafo;
    std::map <int, Airport*> *data = nullptr;

protected:
    static double gradosARadianes(double grados) {
        return grados * (M_PI / 180);
    }

    double obtenerPeso(int origen, int destino) {
        double radioTierra = 6371;
        Airport *airportOrigen = data->find(origen)->second;
        Airport *airportDestino = data->find(destino)->second;
        double distanciaLatitud = gradosARadianes(airportDestino->latitud - airportOrigen->latitud);
        double distanciaLongitud = gradosARadianes(airportDestino->longitud - airportOrigen->longitud);
        double a = sin(distanciaLatitud / 2) * sin(distanciaLatitud / 2) + cos(gradosARadianes(airportOrigen->latitud)) * cos(gradosARadianes(airportDestino->latitud)) * sin(distanciaLongitud / 2) * sin(distanciaLongitud / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        double distancia = radioTierra * c;
        return distancia;
    }

public:
    explicit Graph () : vertices{0}, aristas{0} {}

    static nodoListaAdyacencia* crearNodoListaAdyacencia (int origen, int destino, double peso) {
        auto* nuevoNodo = new nodoListaAdyacencia(origen, destino, peso);
        return nuevoNodo;
    }

    bool buscarArista(int origen, int destino) {
        auto* actual = nodosGrafo[origen]->head;
        while (actual) {
            if (actual->idDestino == destino) {
                return true;
            }
            actual = actual->next;
        }
        return false;
    }

    void cargarData(std::map <int, Airport*>& datos) {
        data = &datos;
    }

    void agregarAristaAeropuerto(int origen, int destino) {
        nodoListaAdyacencia* nuevoNodo = crearNodoListaAdyacencia(origen, destino, obtenerPeso(origen, destino));
        if (!buscarVertice(origen)) {
            nodosGrafo[origen] = new listaAdyacencia();
            nodosGrafo[origen]->head = nullptr;
            vertices++;
        }
        if (!buscarArista(origen, destino) && origen != destino) {
            nuevoNodo->next = nodosGrafo[origen]->head;
            nodosGrafo[origen]->head = nuevoNodo;
            agregarVertice(destino);
            aristas++;
        }

        if (!esDirigido) {
            nuevoNodo = crearNodoListaAdyacencia(destino, origen, obtenerPeso(destino, origen));
            if (!buscarVertice(destino)) {
                nodosGrafo[destino] = new listaAdyacencia();
                nodosGrafo[destino]->head = nullptr;
                vertices++;
            }
            if (!buscarArista(destino, origen) && destino != origen) {
                nuevoNodo->next = nodosGrafo[destino]->head;
                nodosGrafo[destino]->head = nuevoNodo;
            }
        }
    }

    void agregarArista(int origen, int destino, double peso) {
        nodoListaAdyacencia* nuevoNodo = crearNodoListaAdyacencia(origen, destino, peso);
        if (!buscarVertice(origen)) {
            nodosGrafo[origen] = new listaAdyacencia();
            nodosGrafo[origen]->head = nullptr;
            vertices++;
        }
        if (!buscarArista(origen, destino) && origen != destino) {
            nuevoNodo->next = nodosGrafo[origen]->head;
            nodosGrafo[origen]->head = nuevoNodo;
            agregarVertice(destino);
            aristas++;
        }

        if (!esDirigido) {
            nuevoNodo = crearNodoListaAdyacencia(destino, origen, peso);
            if (!buscarVertice(destino)) {
                nodosGrafo[destino] = new listaAdyacencia();
                nodosGrafo[destino]->head = nullptr;
                vertices++;
            }
            if (!buscarArista(destino, origen) && destino != origen) {
                nuevoNodo->next = nodosGrafo[destino]->head;
                nodosGrafo[destino]->head = nuevoNodo;
            }
        }
    }

    bool buscarVertice(int contenidoVertice) {
        auto iterator = nodosGrafo.find(contenidoVertice);
        return iterator != nodosGrafo.end();
    }

    void agregarVertice(int contenidoVertice) {
        if (!buscarVertice(contenidoVertice)) {
            auto *list = new listaAdyacencia();
            nodosGrafo.insert(std::pair<int, listaAdyacencia *>(contenidoVertice, list));
            vertices++;
        }
    }

    void eliminarVertice(int contenidoVertice) {
        if (buscarVertice(contenidoVertice)) {
            for (auto &it : nodosGrafo) {
                nodoListaAdyacencia* actual = it.second->head;
                nodoListaAdyacencia* previo = nullptr;
                while (actual) {
                    if (actual->idDestino == contenidoVertice) {
                        aristas--;
                        if (!previo) {
                            it.second->head = actual->next;
                            delete actual;
                        } else {
                            previo->next = actual->next;
                            delete actual;
                        }
                    }
                    previo = actual;
                    actual = actual->next;
                }
            }
            nodoListaAdyacencia* actual = nodosGrafo[contenidoVertice]->head;
            int restaAristas = 0;
            while (actual) {
                restaAristas++;
                actual = actual->next;
            }
            aristas -= restaAristas;
            nodosGrafo.erase(contenidoVertice);
            vertices--;
        }
    }

    void eliminarArista(int origen, int destino) {
        if (buscarArista(origen, destino)) {
            aristas--;
            nodoListaAdyacencia* actual = nodosGrafo[origen]->head;
            nodoListaAdyacencia* previo = nullptr;
            while (actual) {
                if (actual->idDestino == destino) {
                    if (!previo) {
                        nodosGrafo[origen]->head = actual->next;
                        delete actual;
                    } else {
                        previo->next = actual->next;
                        delete actual;
                    }
                }
                previo = actual;
                actual = actual->next;
            }
        }
    }

    bool esDenso(double cotaDensidad) {
        if (esDirigido) {
            double densidad = ((double)(aristas))/((double)(vertices)*((double)(vertices)-1));
            return densidad >= cotaDensidad;
        } else {
            double densidad = (2*(double)(aristas))/((double)(vertices)*((double)(vertices)-1));
            return densidad >= cotaDensidad;
        }
    }

    bool esConexo() {
        if (esDirigido) {
            bool encontrado = false;
            for (auto & it : nodosGrafo) {
                if (!it.second->head) {
                    for (auto & iterator : nodosGrafo) {
                        encontrado = false;
                        auto *actual = iterator.second->head;
                        while (actual) {
                            if (actual->idDestino == it.first) {
                                encontrado = true;
                                break;
                            }
                            actual = actual->next;
                        }
                        if (encontrado) break;
                    }
                    if (!encontrado) return false;
                }
            }
            return true;
        } else {
            for (auto & it : nodosGrafo) {
                if (!it.second->head) {
                    return false;
                }
            }
            return true;
        }
    }

    void printGraph() {
        for (auto & it : nodosGrafo) {
            nodoListaAdyacencia* pCrawl;
            auto iterator = nodosGrafo.find(it.first);
            if (iterator != nodosGrafo.end()) {
                pCrawl = nodosGrafo[it.first]->head;
            } else {
                pCrawl = nullptr;
            }
            if (pCrawl) {
                std::cout << "\nLista de adyacencia del vertice " << it.first << "\n head ";
            }
            while (pCrawl) {
                std::cout << "-> " << pCrawl->idDestino << "(" << pCrawl->peso << ")";
                pCrawl = pCrawl->next;
            }
        }
        std::cout << "\nVertices: " << vertices << "\nAristas: " << aristas << std::endl;
    }

    ~Graph() {
        for (auto element : nodosGrafo) {
            delete element.second;
        }
    }

};

#endif //PROYECTO_AIRPORTS_GRAPH_H
