# Trabalho Prático 1
## Exercício 1

*Este problema usa optimização MIP (“Mixed Integer Programming” (OrTools) e representação por  Grafos ( NetworkX).*


1. Para um distribuidor de encomendas o seu território está organizados em pontos (“nodes”) de fornecimento (“sources”), pontos de passagem  e pontos de entrega (“sinks”) ligados por vias de comunicação (“edges”) bidirecionais cada uma das quais associada uma capacidade em termos do número de veículos de transporte que suporta.
2. Os items distribuidos estão organizados em “pacotes” de três tipos “standard” : uma unidade, duas unidades e cinco unidades. Os pacotes são transportados em veículos todos com a capacidade de 10 unidades. Cada ponto de fornecimento tem um limite no número total de unidades que tem em “stock” e um limite no número de veículos que dispõe.
3. Cada encomenda é definida por o identificador do ponto de entrega e pelo número de pacotes, de cada um dos tipos, que devem ser entregues nesse ponto.
4. O objetivo do problema é decidir, a partir de uma encomenda e com um mínimo no número de veículos,
        - *em cada* *ponto de fornecimento,  se estará envolvido no fornecimento de unidades que essa encomenda requer sem violar os limites do seu “stock”.*
        - *em cada ponto de fornecimento,   como empacotar as unidades disponíveis, de acordo com a encomenda”,  e como as distribuir por veículos,*
        - *em cada veículo,* *qual o percurso a seguir até  ao ponto de entrega; para cada via ao longo de cada percurso, o total de veículos não pode exceder a capacidade dessa via.*


| Efectue um (ou mais!)  dos seguintes exercícios |

## Exercício 2

*Este problema deve usar a optimização CP (“Constraint Programming”) no OrTools e procura implementar soluções de uma generalização do problema Sudoku.*

A [definição usual do problema Sudoku](https://en.wikipedia.org/wiki/Sudoku) (extraido da Wikipedia) contém a seguinte definição

>  In classic Sudoku, the objective is to fill a 9 × 9 grid with digits so that each column, each row, and each of the nine 3 × 3 subgrids that compose the grid (also called "boxes", "blocks", or "regions") contains all of the digits from 1 to 9. The puzzle setter provides a partially completed grid, which for a [well-posed](https://en.wikipedia.org/wiki/Well-posed_problem) puzzle has a single solution.

Neste trabalho pretende-se generalizar o problema em várias direções:

    - Em primeiro lugar a grelha tem como parâmetro fundamental um inteiro que toma vários valores $$\,n \in \{3,4,...\}$$. Fundamentalmente a grelha passa de um quadrado com $$\,n^2\times n^2\,$$ células  para um cubo tridimensional  de dimensões $$\, n^2\times n^2 \times n^2\,$$. Cada posição na grelha  é representada por um triplo de inteiros $$\,(i,j,k)\in \{1..n^2\}^3\,$$.
    - Em segundo lugar as “regiões” que a definição menciona deixam de ser linhas, colunas e “sub-grids” para passar a ser  qualquer “box”  genérica com um número de células $$\,\leq n^3\,$$. Cada “box” é representado por um dicionário $$\,D\,$$ que associa, no estado inicial,  cada posição  $$\,(i,j,k)\in D\,$$ na “box” a um valor  inteiro no intervalo  $$\,\{0..n^3\}\,$$. 
    - Na inicialização da solução  as células associadas ao valor $$\,0\,$$ estão livres para ser instanciadas com qualquer valor não nulo. Se nessa fase, uma célula está associada a um valor não-nulo, então esse valor está fixo e qualquer solução do problema não o modifica. 
    - A solução final do problema, tal como no problema original,  verifica uma restrição do tipo **all-different**  que, neste caso, tem a forma
    > dentro uma mesma “box”, todas as células têm valores distintos no intervalo $$\,\{1..n^3\}\,$$.
    - Considera-se neste problema suas formas básicas de “boxes”:
        - “cubos”  de $$n^3\,$$ células determinados pelo seu vértice superior, anterior, esquerdo
        - “paths” determinados pelo seu vértice de início, o vértice final e pela ordem entre os índices dos vértices sucessivos.
        O “input” do problema é um conjunto de “boxes” e um conjunto de alocações de valores a células. 