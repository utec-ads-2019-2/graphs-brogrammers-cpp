#include <bits/stdc++.h>

using namespace std;

struct nodoListaAdyacencia {
    int data;
    nodoListaAdyacencia* next;
};

struct listaAdyacencia {
    nodoListaAdyacencia *head;
};

class Graph {
    private:
        int vertices;
        map <int, listaAdyacencia*> nodosGrafo;

    public:
        Graph (int vertices) {
            this->vertices = vertices;
        }
        
        nodoListaAdyacencia* crearNodoListaAdyacencia (int destino) {
            nodoListaAdyacencia* nuevoNodo = new nodoListaAdyacencia;
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
                    cout << "\nAdjacency list of vertex " << v << "\n head ";
                }
                while (pCrawl) {
                    cout<<"-> "<<pCrawl->data;
                    pCrawl = pCrawl->next;
                }
            }
        }
};

int main() {
    Graph gh(5);
    gh.addEdge(0, 1);
    gh.addEdge(1, 2);
    gh.addEdge(2, 3);
 
    gh.printGraph();
 
    return 0;
}
