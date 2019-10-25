#include <iostream>
#include <cstdlib>

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
        listaAdyacencia* nodosGrafo;

    public:
        Graph(int vertices) {
            this->vertices = vertices;
            nodosGrafo = new listaAdyacencia [vertices];
            for (int i = 0; i < vertices; ++i)
                nodosGrafo[i].head = NULL;
        }
        
        nodoListaAdyacencia* newnodoListaAdyacencia(int destino) {
            nodoListaAdyacencia* nuevoNodo = new nodoListaAdyacencia;
            nuevoNodo->data = destino;
            nuevoNodo->next = NULL;
            return nuevoNodo;
        }
        
        void addEdge(int origen, int destino) {
            nodoListaAdyacencia* nuevoNodo = newnodoListaAdyacencia(destino);
            nuevoNodo->next = nodosGrafo[origen].head;
            nodosGrafo[origen].head = nuevoNodo;
            // Esta parte determina si es dirigido o no
            nuevoNodo = newnodoListaAdyacencia(origen);
            nuevoNodo->next = nodosGrafo[destino].head;
            nodosGrafo[destino].head = nuevoNodo;
        }
        
        void printGraph() {
            int v;
            for (v = 0; v < vertices; ++v) {
                nodoListaAdyacencia* pCrawl = nodosGrafo[v].head;
                if (pCrawl) {
                    cout << "\nAdjacency list of vertex " << v << "\n head ";
                }
                while (pCrawl) {
                    cout<<"-> "<<pCrawl->data;
                    pCrawl = pCrawl->next;
                }
                //if (pCrawl) cout << endl << endl;
            }
        }
};

int main() {
    Graph gh(5);
    gh.addEdge(0, 1);
    gh.addEdge(1, 0);
 
    gh.printGraph();
 
    return 0;
}