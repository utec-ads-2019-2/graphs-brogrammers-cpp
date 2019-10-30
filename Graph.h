#ifndef PROYECTO_AIRPORTS_GRAPH_H
#define PROYECTO_AIRPORTS_GRAPH_H

#include <cmath>
#include <iostream>
#include <map>
#include <algorithm>
#include <queue>
#include <list>
#include <iomanip>

#include "Edge.h"
#include "Airport.h"

class Graph {
private:
    int vertices;
    int aristas;
    bool esDirigido = false;
    std::map <int, listaAdyacencia*> nodosGrafo;
    std::map <int, Airport*> *data = nullptr;

    std::list <std::pair <int, double>> *adyacenciaPrim{};

    std::map <int, int> padreKruskal;
    std::vector <std::pair <double, std::pair <int, int>>> nodosKruskal;
    std::vector <std::pair <double, std::pair <int, int>>> arbolMinimaExpansion;

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

    void executeDFS(int id_nodo, std::map <int, bool> &map_nodes_visited){
        map_nodes_visited[id_nodo] = true;
        //std::cout << id_nodo << " ";
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
            map_nodes_visited[element.first] =  false;
        }
        //std::cout << "Componentes conectados: \n";
        for(auto const& element : nodosGrafo){
            if(!map_nodes_visited[element.first]){
                executeDFS(element.first, map_nodes_visited);
                //std::cout << "\n";
                num_connected++;
            }
        }
        std::cout << "Conexo: ";
        return num_connected == 1;
    }

    bool checkBipartito(){
        std::map<int,int> map_nodos_color;
        for(auto const& element : nodosGrafo){
            map_nodos_color[element.first] =  -1;
        }
        map_nodos_color[nodosGrafo.begin()->first] = 1;

        std::queue<int> q_nodes;
        q_nodes.push(nodosGrafo.begin()->first);

        while(!q_nodes.empty()){
            int u = q_nodes.front();
            q_nodes.pop();
            auto* actual = nodosGrafo[u]->head;
            while(actual){
                if(map_nodos_color[actual->idDestino] == -1){
                    map_nodos_color[actual->idDestino] = 1 - map_nodos_color[u];
                    q_nodes.push(actual->idDestino);
                }
                else if(map_nodos_color[actual->idDestino] == map_nodos_color[u]){
                    return false;
                }
                actual = actual->next;
            }
        }
        return true;
    }

    void executeDFS_SC(int id_nodo, std::map<int, bool> &map_nodes_visited){
        map_nodes_visited[id_nodo] = true;
        auto* actual = nodosGrafo[id_nodo]->head;
        while (actual) {
            if(!map_nodes_visited[actual->idDestino]){
                executeDFS_SC(actual->idDestino, map_nodes_visited);
            }
            actual = actual->next;
        }
    }

    void getTranspose(Graph &rgraph){
        for(auto const& element : nodosGrafo){
            auto* actual = element.second->head;
            while(actual){
                rgraph.agregarArista(actual->idDestino, element.first, 1);
                actual = actual->next;
            }
        }
        //std::cout << "Reverse Graph \n";
        //rgraph.printGraph();
    }

    bool isStronglyConnected(){
        std::map<int, bool> map_nodes_visited;
        for(auto const& element : nodosGrafo){
            map_nodes_visited[element.first] =  false;
        }
        executeDFS_SC(nodosGrafo.begin()->first, map_nodes_visited);

        for(auto const& element : nodosGrafo){
            if(!map_nodes_visited[element.first]){
                std::cout << "Failed first DFS \n";
                return false;
            }
        }
        Graph reversedGraph;
        getTranspose(reversedGraph);
        for(auto const& element : nodosGrafo){
            map_nodes_visited[element.first] =  false;
        }
        reversedGraph.executeDFS_SC(reversedGraph.nodosGrafo.begin()->first, map_nodes_visited);
        for(auto const& element : nodosGrafo){
            if(!map_nodes_visited[element.first]){
                return false;
            }
        }
        return true;
    }

    static nodoListaAdyacencia* crearNodoListaAdyacencia (int origen, int destino, double peso) {
        auto* nuevoNodo = new nodoListaAdyacencia(destino, peso);
        return nuevoNodo;
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

    void imprimirKruskal(Graph &grafoResultado) {
        //std::cout << "Arista:   " << "   Peso" << std::endl;
        for (auto & it : arbolMinimaExpansion) {
            grafoResultado.agregarArista(it.second.first, it.second.second, it.first);
            //std::cout << it.second.first << "  -  " << it.second.second << "   :   " << std::setprecision(10) << it.first << std::endl;
        }
    }

    void agregarALista() {
        for (auto & it : nodosGrafo) {
            auto *actual = it.second->head;
            while (actual) {
                adyacenciaPrim[it.first].emplace_back(actual->idDestino, actual->peso);
                actual = actual->next;
            }
        }
    }

public:
    explicit Graph () : vertices{0}, aristas{0} {}

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
            return checkComponentesConectados();
        }
        else{
            throw std::invalid_argument("Propiedad conexo solo valido para grafos no dirigidos");
        }
    }

    bool esBipartito(){
        return checkBipartito();
    }

    bool esFuertementeConexo(){
        if(esDirigido){
            return isStronglyConnected();
        }
        else{
            throw std::invalid_argument("Propiedad fuertemente conexo solo valido para grafos dirigidos");
        }
    }

    void algoritmoPrim(Graph &grafoResultado) {
        if (!esDirigido && this->esConexo()) {
            int maximoId = nodosGrafo.rend()->first + 1;
            adyacenciaPrim = new std::list<std::pair <int, double>> [maximoId];
            agregarALista();
            std::priority_queue <std::pair <int, int>, std::vector <std::pair <int, int>>, std::greater<std::pair <int, int>> > colaPrioridad;
            int puntoInicio = nodosGrafo.begin()->first;
            std::vector <double> clave(maximoId, 2147483647);
            std::map <int, int> arrayPadre;
            std::vector <bool> estaEnMST(maximoId, false);
            colaPrioridad.push(std::make_pair(0, puntoInicio));
            clave[puntoInicio] = 0;
            while (!colaPrioridad.empty()) {
                int verticeMenor = colaPrioridad.top().second;
                colaPrioridad.pop();
                estaEnMST[verticeMenor] = true;
                for (auto & it : adyacenciaPrim[verticeMenor]) {
                    int primerDestino = it.first;
                    double peso = it.second;
                    if (!estaEnMST[primerDestino] && clave[primerDestino] > peso) {
                        clave[primerDestino] = peso;
                        colaPrioridad.push(std::make_pair(clave[primerDestino], primerDestino));
                        arrayPadre[primerDestino] = verticeMenor;
                    }
                }
            }
            for (auto & iterador : arrayPadre) {
                grafoResultado.agregarArista(iterador.second, iterador.first, clave[iterador.first]);
                //std::cout << iterador.second << " - " << iterador.first << " - " << clave[iterador.first] << std::endl;
            }
        } else {
            throw std::invalid_argument("Algoritmo Prim no puede ser aplicado sobre grafos dirigidos o no conexos");
        }
    }

    void algoritmoKruskal(Graph &grafoResultado) {
        if (!esDirigido) {
            int i, representanteA, representanteB;
            construirConjunto();
            std::sort(nodosKruskal.begin(), nodosKruskal.end());
            for (auto &it : nodosKruskal) {
                representanteA = encontrarConjunto(it.second.first);
                representanteB = encontrarConjunto(it.second.second);
                if (representanteA != representanteB) {
                    arbolMinimaExpansion.push_back(it);
                    unionSet(representanteA, representanteB);
                }
            }
            imprimirKruskal(grafoResultado);
        } else {
            throw std::invalid_argument("Algoritmo Kruskal no puede ser aplicado sobre grafos dirigidos");
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
            element.second->head->killSelf();
            element.second->head = nullptr;
        }
        for (auto element : nodosGrafo) {
            delete element.second;
        }
    }

};

#endif //PROYECTO_AIRPORTS_GRAPH_H