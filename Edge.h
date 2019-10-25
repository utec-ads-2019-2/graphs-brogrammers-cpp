#ifndef PROYECTO_AIRPORTS_EDGE_H
#define PROYECTO_AIRPORTS_EDGE_H

struct nodoListaAdyacencia {
    int data;
    nodoListaAdyacencia* next;
};

struct listaAdyacencia {
    nodoListaAdyacencia *head;
};

#endif //PROYECTO_AIRPORTS_EDGE_H