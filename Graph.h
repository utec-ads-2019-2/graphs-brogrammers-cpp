#ifndef PROYECTO_AIRPORTS_GRAPH_H
#define PROYECTO_AIRPORTS_GRAPH_H

#include <iostream>
#include <map>
#include <algorithm>

#include "Edge.h"
#include "Airport.h"

class Graph {
private:
    int vertices;
    int aristas;
    bool esDirigido = false;
    std::map <int, int> padreKruskal;
    std::vector <std::pair <int, std::pair <int, int>>> nodosKruskal;
    std::vector <std::pair <int, std::pair <int, int>>> arbolMinimaExpansion;
    std::map <int, listaAdyacencia*> nodosGrafo;
    std::map <int, Airport*> *data = nullptr;

    void executeDFS(int id_nodo, std::map<int, bool> &map_nodes_visited){
        map_nodes_visited[id_nodo] = true;
        std::cout << id_nodo << " ";
        //Recursividad para todos los vertices adyacenetes a este nodo
        auto* actual = nodosGrafo[id_nodo]->head;
        while (actual) {
            if(!map_nodes_visited[actual->idDestino]){
                executeDFS(actual->idDestino, map_nodes_visited);
            }
            actual = actual->next;
        }
    }

    bool checkComponentesConectados(){
        std::map<int, bool> map_nodes_visited;
        int num_connected = 0;
        for(auto const& element : nodosGrafo){
            map_nodes_visited[element.first] =  false;		// init the bool map with keys of nodos to false;
        }
        std::cout << "Componentes conectados: \n";
        for(auto const& element : nodosGrafo){
            if(!map_nodes_visited[element.first]){
                executeDFS(element.first, map_nodes_visited); // imprime todos los nodos alcanzables desde el nodo 'element.first'
                std::cout << "\n";
                num_connected++;
            }
        }
        std::cout << "Conexo: ";
        return num_connected == 1;  // si hay 1 componente conectado return true, si hay 2 o mas componentes conectados return false
    }

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
        auto* nuevoNodo = new nodoListaAdyacencia(destino, peso);
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
        if(!esDirigido){
            std::cout << "Grafo No Dirigido \n";
            return checkComponentesConectados();
        }
        else{
            std::cout << "Grafo Dirigido \n";
            return checkComponentesConectados();
        }
    }

    static nodoMinHeap* nuevoNodoMinHeap(int valor, double clave) {
        auto *nuevoNodo = new nodoMinHeap(valor, clave);
        return nuevoNodo;
    }

    static minHeap* crearMinHeap(int capacidad) {
        auto *nuevoMinHeap = new minHeap(0, capacidad);
        return nuevoMinHeap;
    }

    static void swapNodoMinHeap(nodoMinHeap **a, nodoMinHeap **b) {
        nodoMinHeap *t = *a;
        *a = *b;
        *b = t;
    }

    static void minimoHeapify(minHeap *minimoHeap, int indice) {
        int menor = indice, izquierda = 2 * indice + 1, derecha = 2 * indice + 2;
        auto *nodoDerecha1 = minimoHeap->array.find(derecha)->second;
        auto *nodoMenor1 = minimoHeap->array.find(menor)->second;
        if (izquierda < minimoHeap->numeroNodos && nodoDerecha1->clave < nodoMenor1->clave) {
            menor = izquierda;
        }
        if (derecha < minimoHeap->numeroNodos && nodoDerecha1->clave < nodoMenor1->clave) {
            menor = derecha;
        }
        if (menor != indice) {
            auto *nodoMenor = minimoHeap->array.find(menor)->second;
            auto *nodoIndice = minimoHeap->array.find(indice)->second;
            minimoHeap->posicion.find(nodoMenor->valor)->second = indice;
            minimoHeap->posicion.find(nodoIndice->valor)->second = menor;
            swapNodoMinHeap(&minimoHeap->array.find(menor)->second, &minimoHeap->array.find(indice)->second);
            minimoHeapify(minimoHeap, menor);
        }
    }

    static int estaVacio(minHeap *minimoHeap) {
        return minimoHeap->numeroNodos == 0;
    }

    static nodoMinHeap* extraerMinimo(minHeap *minimoHeap) {
        if (estaVacio(minimoHeap)) {
            return nullptr;
        }
        nodoMinHeap *raiz = minimoHeap->array.begin()->second;
        nodoMinHeap *ultimoNodo = minimoHeap->array.rbegin()->second;
        minimoHeap->array.begin()->second = ultimoNodo;
        minimoHeap->posicion.find(raiz->valor)->second = minimoHeap->numeroNodos - 1;
        minimoHeap->posicion.find(ultimoNodo->valor)->second = 0;
        --minimoHeap->numeroNodos;
        minimoHeapify(minimoHeap, 0);
        return raiz;
    }

    static void disminuirClave(minHeap *minimoHeap, int valor, double clave) {
        int i = minimoHeap->posicion.find(valor)->second;
        minimoHeap->array.find(i)->second->clave = clave;
        while (i && minimoHeap->array.find(i)->second->clave < minimoHeap->array.find((i - 1) / 2)->second->clave) {
            minimoHeap->posicion.find(minimoHeap->array.find(i)->second->valor)->second = (i - 1) / 2;
            minimoHeap->posicion.find(minimoHeap->array.find((i - 1) / 2)->second->valor)->second = i;
            swapNodoMinHeap(&minimoHeap->array.find(i)->second, &minimoHeap->array.find((i - 1) / 2)->second);
            i = (i - 1) / 2;
        }
    }

    static bool estaEnMinHeap(minHeap *minimoHeap, int valor) {
        return minimoHeap->posicion.find(valor)->second < minimoHeap->numeroNodos;
    }

    static void imprimirMapa(const std::map <int, int>& resultado) {
        for (auto & it : resultado) {
            std::cout << it.second << " - " << it.first << std::endl;
        }
    }

    void algoritmoPrim() {
        int tamGrafo = vertices;
        std::map<int, int> padre;
        std::map<int, double> clave;
        auto *minimoHeap = crearMinHeap(tamGrafo);
        for (auto it = ++nodosGrafo.begin(); it != nodosGrafo.end(); ++it) {
            clave[it->first] = INT_MAX;
        }
        for (auto it = ++nodosGrafo.begin(); it != nodosGrafo.end(); ++it) {
            padre[it->first] = -1;
            auto *nodoNuevo = nuevoNodoMinHeap(it->first, clave.find(it->first)->second);
            minimoHeap->array.insert({it->first, nodoNuevo});
            minimoHeap->posicion.insert({it->first, it->first});
        }
        clave[0] = 0;
        auto it = nodosGrafo.begin();
        minimoHeap->array.begin()->second = nuevoNodoMinHeap(it->first, clave.find(it->first)->second);
        minimoHeap->posicion.begin()->second = 0;
        minimoHeap->numeroNodos = tamGrafo;
        while (!estaVacio(minimoHeap)) {
            auto *nodoMinimo = extraerMinimo(minimoHeap);
            int verticeTemporal = nodoMinimo->valor;
            auto *pCrawl = nodosGrafo[verticeTemporal]->head;
            while (pCrawl) {
                int destinoTemporal = pCrawl->idDestino;
                if (estaEnMinHeap(minimoHeap, destinoTemporal) && pCrawl->peso < clave[destinoTemporal]) {
                    clave[destinoTemporal] = pCrawl->peso;
                    padre[destinoTemporal] = verticeTemporal;
                    disminuirClave(minimoHeap, destinoTemporal, clave[destinoTemporal]);
                }
                pCrawl = pCrawl->next;
            }
        }
        imprimirMapa(padre);
    }

    void agregarAVector(int origen, listaAdyacencia *&nodo) {
        auto *actual = nodo->head;
        while (actual) {
            nodosKruskal.emplace_back(actual->peso, std::make_pair(origen, actual->idDestino));
            actual = actual->next;
        }
    }

    void construirConjunto() {
        for (auto & it : nodosGrafo) {
            padreKruskal[it.first] = it.first;
            agregarAVector(it.first, it.second);
        }
    }

    int encontrarConjunto(int nodo) {
        if (nodo == padreKruskal[nodo]) {
            return nodo;
        } else {
            return encontrarConjunto(padreKruskal[nodo]);
        }
    }

    void unionSet(int nodoA, int nodoB) {
        padreKruskal[nodoA] = padreKruskal[nodoB];
    }

    void algoritmoKruskal() {
        int i, representanteA, representanteB;
        construirConjunto();
        std::sort(nodosKruskal.begin(), nodosKruskal.end());
        for (auto & it : nodosKruskal) {
            representanteA = encontrarConjunto(it.second.first);
            representanteB = encontrarConjunto(it.second.second);
            if (representanteA != representanteB) {
                arbolMinimaExpansion.push_back(it);
                unionSet(representanteA, representanteB);
            }
        }
        imprimirKruskal();
    }

    void imprimirKruskal() {
        std::cout << "Arista:   " << "   Peso" << std::endl;
        for (auto & it : arbolMinimaExpansion) {
            std::cout << it.second.first << "  -  " << it.second.second << "   :   " << it.first << std::endl;
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
