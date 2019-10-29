#ifndef PROYECTO_AIRPORTS_GRAPH_H
#define PROYECTO_AIRPORTS_GRAPH_H

#include <iostream>
#include <map>
#include <algorithm>
#include <queue>
#include <list>

#include <graphics.h>
#include <cmath>
#include <conio.h>

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
    std::vector <std::pair <int, std::pair <int, int>>> nodosKruskal;
    std::vector <std::pair <int, std::pair <int, int>>> arbolMinimaExpansion;

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

    bool checkBipartito(){
        std::map<int,int> map_nodos_color;      // '1' used for first color and '0' for second color
        for(auto const& element : nodosGrafo){
            map_nodos_color[element.first] =  -1;		// init all nodes in the map_color with 'No colored'
        }
        map_nodos_color[nodosGrafo.begin()->first] = 1;

        std::queue<int> q_nodes;
        q_nodes.push(nodosGrafo.begin()->first); //Encolamos el primer id_node y go for BFS traversal

        while(!q_nodes.empty()){
            int u = q_nodes.front();    // sacamos de la cola
            q_nodes.pop();
            auto* actual = nodosGrafo[u]->head;
            //Busca todos los nodos No coloreados adyacentes a 'u'
            while(actual){
                if(map_nodos_color[actual->idDestino] == -1){   // si el destino No esta coloreado
                    map_nodos_color[actual->idDestino] = 1 - map_nodos_color[u];
                    q_nodes.push(actual->idDestino);    //ponemos en cola el nodo que acabamos de colorear
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
        //std::cout << id_nodo << " ";
        //Recursividad para todos los vertices adyacentes a este nodo
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
                rgraph.agregarArista(actual->idDestino, element.first, 1);  // Generate Transpose Graph
                actual = actual->next;
            }
        }
        //std::cout << "Reverse Graph \n";
        //rgraph.printGraph();
    }

    bool isStronglyConnected(){
        std::map<int, bool> map_nodes_visited;
        for(auto const& element : nodosGrafo){
            map_nodes_visited[element.first] =  false;		// init the bool map with keys of nodos to false;
        }
        executeDFS_SC(nodosGrafo.begin()->first, map_nodes_visited); // DFS traversal for first node

        for(auto const& element : nodosGrafo){          //If DFS did not visit all nodes return false
            if(!map_nodes_visited[element.first]){
                std::cout << "Failed first DFS \n";
                return false;
            }
        }
        // Create reversed graph
        Graph reversedGraph;
        getTranspose(reversedGraph);
        //Set everything to false again for second DFS in the reversed graph
        for(auto const& element : nodosGrafo){
            map_nodes_visited[element.first] =  false;		// init the bool map with keys of nodos to false;
        }
        //executeDFS_SC(reversedGraph.nodosGrafo.begin()->first, map_nodes_visited);
        reversedGraph.executeDFS_SC(reversedGraph.nodosGrafo.begin()->first, map_nodes_visited);
        //Last: if DFS did not visit all nodes return false
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

    void imprimirKruskal() {
        std::cout << "Arista:   " << "   Peso" << std::endl;
        for (auto & it : arbolMinimaExpansion) {
            std::cout << it.second.first << "  -  " << it.second.second << "   :   " << it.first << std::endl;
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
            std::cout << "Grafo No Dirigido \n";
            return checkComponentesConectados();
        }
        else{
            std::cout << "Grafo Dirigido \n";
            return checkComponentesConectados();
        }
    }

    bool esBipartito(){
        std::cout << "Bipartito: ";
        return checkBipartito();
    }

    bool esFuertementeConexo(){
        std::cout << "Fuertemente Conexo: ";
        if(esDirigido){
            return isStronglyConnected();
        }
        else{
            std::cout << "(Es un grafo No Dirigido) ";
            return false;
        }
    }

    void algoritmoPrim() {
        if (!esDirigido) {
            int maximoId = nodosGrafo.rend()->first + 1;
            adyacenciaPrim = new std::list<std::pair <int, double>> [maximoId];
            agregarALista();
            std::priority_queue <std::pair <int, int>, std::vector <std::pair <int, int>>, std::greater<> > colaPrioridad;
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
                std::cout << iterador.second << " - " << iterador.first << " - " << clave[iterador.first] << std::endl;
            }
        } else {
            throw std::invalid_argument("Algoritmo Prim no puede ser aplicado sobre grafos dirigidos");
        }
    }

    void algoritmoKruskal() {
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
            imprimirKruskal();
        } else {
            throw std::invalid_argument("Algoritmo Kruskal no puede ser aplicado sobre grafos dirigidos");
        }
    }

    void graficar() {
        initwindow(800, 600);
        int x, y;
        line(0, 300, getmaxx(), 300);
        line(400, 0, 400, getmaxy());
        float pi = 3.1415;
        for (int i = -360; i <= 360; ++i) {
            x = (int)400+i;
            y = (int)300-sin(i*pi/100)*25;
            putpixel(x, y, WHITE);
        }
        getch();
        closegraph();
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
        delete adyacenciaPrim;
    }

};

#endif //PROYECTO_AIRPORTS_GRAPH_H
