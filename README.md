# Projeto de Estrutura de Dados — Algoritmo de Dijkstra

Projeto acadêmico de Estrutura de Dados (Grupo 2) que apresenta uma implementação em linguagem C do algoritmo de Dijkstra para calcular caminhos de menor custo em um grafo ponderado representado por matriz de adjacência.

## Objetivo

Explorar conceitos de grafos, representação por matriz de adjacência, busca do menor caminho e rastreamento de predecessores.

## Tecnologias e conceitos

- **Linguagem C**
- Grafos ponderados.
- Matriz de adjacência.
- Algoritmo de Dijkstra.
- Vetores para distâncias, nós visitados e predecessores.

## Arquivos

- `dijkstrac.c` — implementação do algoritmo e exemplo de grafo com cinco vértices.
- `PROJETO ESTRUTURA DE DADOS GRUPO 2.docx` — material escrito do trabalho.
- `PROJETO ESTRUTURA DE DADOS.pptx` — apresentação do trabalho.

## Como compilar e executar

É necessário ter um compilador C, como GCC.

Em ambientes Linux ou macOS com GCC:

```bash
gcc -std=c99 -Wall -Wextra dijkstrac.c -o dijkstra
./dijkstra
```

O programa utiliza a função `sleep()` da biblioteca `unistd.h`; em ambientes Windows, pode ser necessário adaptar essa parte ou utilizar um ambiente compatível, como WSL.

## Observação técnica

Este repositório registra uma implementação acadêmica inicial. A lógica de caminho e de backup de rotas ainda precisa de revisão e testes antes de ser considerada uma implementação robusta do algoritmo de Dijkstra. O código atual utiliza um grafo de exemplo fixo, com cinco vértices, e não recebe grafos por entrada do usuário.

## Melhorias planejadas

- Tratar corretamente vértices inalcançáveis.
- Inicializar e validar todos os índices e predecessores.
- Revisar a lógica de backup, separando-a do algoritmo clássico de Dijkstra.
- Exibir o caminho completo, incluindo origem e destino.
- Adicionar testes com diferentes grafos e casos-limite.
- Melhorar a portabilidade para Windows.

---

Projeto acadêmico de [Luis Felipe](https://github.com/Luis9999999).
