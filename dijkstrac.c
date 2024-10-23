#include <stdio.h>
#include <limits.h>     // Define constantes como INT_MAX para representar infinito
#include <stdbool.h>    // Define o tipo bool para trabalhar com valores booleanos
#include <unistd.h>     // Biblioteca para a função sleep, que adiciona pausas no programa

#define N 5             // Define o número de nós como 5 (nós de A a E)
#define INF INT_MAX     // Define "infinito" como o maior valor de int (usado para representar distâncias infinitas)

// Função para encontrar o nó com a menor distância que ainda não foi visitado
int minDistance(int dist[], bool visited[]) {
    int min = INF, min_index;

    // Loop para percorrer todos os nós e encontrar o de menor distância que ainda não foi visitado
    for (int v = 0; v < N; v++) {
        if (!visited[v] && dist[v] <= min) {
            min = dist[v];        // Atualiza o menor valor de distância encontrado
            min_index = v;        // Armazena o índice do nó com a menor distância
        }
    }
    return min_index;  // Retorna o índice do nó com a menor distância
}

// Função para imprimir o caminho encontrado
void printPath(int parent[], int j) {
    if (parent[j] == -1)
        return;  // Caso base: quando chegar na origem (parent == -1), retorna

    // Chamada recursiva para continuar imprimindo o caminho na ordem correta
    printPath(parent, parent[j]);
    printf("%d ", j);  // Imprime o nó atual do caminho
}

// Função principal que executa o algoritmo de Dijkstra
void dijkstra(int graph[N][N], int src, int target) {
    int distancia[N];        // Array para armazenar a menor distância de cada nó
    bool visita[N];    // Array para marcar quais nós já foram visitados
    int parent[N];      // Array para armazenar o caminho até cada nó
    int backup_distancia[N]; // Array para armazenar a distância de backup (caso precise voltar para uma rota anterior)
    int backup_parent[N]; // Array para armazenar o caminho de backup

    // Inicializa os arrays de distâncias, visitados e pais
    for (int i = 0; i < N; i++) {
        parent[src] = -1;        // Define que o nó de origem não tem pai :C
        distancia[i] = INF;           // Inicializa todas as distâncias com infinito (exceto a origem)
        visita[i] = false;      // Nenhum nó foi visitado no início
        backup_distancia[i] = INF;    // Inicializa as distâncias de backup com infinito
        backup_parent[i] = -1;   // Inicializa os pais de backup
    }

    distancia[src] = 0;               // A distância até a origem (nó src) é 0
    backup_distancia[src] = 0;        // A distância de backup da origem também é 0

    // Loop que encontra o menor caminho para todos os nós
    for (int count = 0; count < N - 1; count++) {
        int u = minDistance(distancia, visita);  // Encontra o nó com a menor distância
        visita[u] = true;                   // Marca o nó como visitado

        // Se o nó atual for o destino, encerra o loop
        if (u == target) {
            printf("Destino alcançado.\n");
            break;
        }

        // Loop para verificar todos os nós adjacentes ao nó u
        for (int v = 0; v < N; v++) {
            // Verifica se:
            // 1. v não foi visitado
            // 2. Há uma aresta entre u e v
            // 3. O valor da distância atual não é infinito
            if (!visita[v] && graph[u][v] && distancia[u] != INF) {
                int nova_distancia = distancia[u] + graph[u][v];  // Calcula a nova distância potencial para o nó v
                
                // Se a nova distância for menor que a distância atual para v
                if (nova_distancia < distancia[v]) {
                    printf("Mudanca de rota: de %d para %d\n", u, v);  // Informa a mudança de rota
                    parent[v] = u;     // Atualiza o pai do nó v para o nó u
                    distancia[v] = nova_distancia;  // Atualiza a menor distância para o nó v
                    backup_distancia[v] = nova_distancia;  // Atualiza a distância de backup
                    backup_parent[v] = parent[v];  // Atualiza o pai de backup
                    printf("Distancia percorrida ate agora: %d\n", distancia[v]);  // Imprime a nova distância percorrida
                    sleep(1);  // Espera 1 segundo antes de continuar
                } else if (nova_distancia > backup_distancia[v]) {
                    // Se o novo caminho for maior que o caminho de backup, volta para a rota anterior
                    printf("Distancia maior detectada! Voltando para o caminho anterior de %d para %d\n", u, backup_parent[u]);
                    distancia[v] = backup_distancia[v];      // Retorna à menor distância (rota de backup)
                    parent[v] = backup_parent[v];  // Volta para o pai de backup
                    sleep(1);  // Espera 1 segundo
                }
            }
        }
    }

    // Imprime o menor caminho final do nó de origem até o destino
    printf("Menor caminho de %d ate %d: ", src, target);
    printPath(parent, target);  // Chama a função printPath para mostrar o caminho
    printf("\nDistancia total: %d\n", distancia[target]);  // Imprime a distância total do menor caminho
}

int main() {
    // Matriz de adjacência representando o grafo com as distâncias entre os nós
    int graph[N][N] = {
        {0, 10, 0, 30, 100},  // A (0) -> B: 10, D: 30, E: 100
        {10, 0, 50, 0, 0},    // B (1) -> A: 10, C: 50
        {0, 50, 0, 20, 10},   // C (2) -> B: 50, D: 20, E: 10
        {30, 0, 20, 0, 60},   // D (3) -> A: 30, C: 20, E: 60
        {100, 0, 10, 60, 0}   // E (4) -> A: 100, C: 10, D: 60
    };

    int origem = 0;  // Define o nó de origem (A)
    int destino = 4; // Define o nó de destino (E)

    dijkstra(graph, origem, destino);  // Chama o algoritmo de Dijkstra para encontrar o menor caminho de A até E

    return 0;
}
