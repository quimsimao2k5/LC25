# Trabalho Prático 2

Este dois exercícios são apresentados como introdução à modelação de problemas através de “SMT solvers” (Z3, CVC5,…) via uma das interfaces Python  (*z3-solver*  ou *pySMT* ) *.*

## Exercício 1

Neste exercício vai-se considerar a modelação de circuitos do 2º grau com falhas como estão descritos [neste  documento](/scl/fi/g89i6nx8zhrasexhxnpxg/Trabalho-pr-tico_-circuitos-com-falhas.paper?rlkey=l98chbu31prxfbzc9xezvd2so&dl=0) . Recomenda-se a leitura atenta desse documento antes de considerar o resto do enunciado deste exercício.

Construa uma resolução das seguintes questões a partir de “inputs” do problema: os parâmetros $$\,\kappa\,,\,n\,$$ e de probabilidade de falha $$\,\varepsilon\,$$ restrita apenas às “gates” **and**.


1. Construa algoritmos para, sob “inputs” do segredo $$\,z\in\{0,1\}^n\,$$ e da “chave mestra” $$\,s\in\{0,1\}^\kappa\,$$, construa o circuito. Adicionalmente a partir deste circuito , construa o modelo SMT do circuito com falhas.
2. Usando o modelo acima, tente construir uma possível estimativa  para $$z$$ numa execução com falhas não nulas; isto é, encontrar
    1.   $$\,z'\in \{0,1\}^n\,$$  que é raíz de todos os polinómios que formam o circuito e 
    2. uma situação de falhas não nulas em “gates” **and**  que conduz a essa estimativa.
3. Conhecido  $$\,z\in\{0,1\}^n\,$$ pretende-se maximizar a probabilidade de falhas **and**  sem que o “output” $$\,0^n\,$$ seja alterado.



## Exercício 2

Considere o problema descrito no documento [+Lógica Computacional: Multiplicação de Inteiros](https://paper.dropbox.com/doc/Logica-Computacional-Multiplicacao-de-Inteiros-n1G7pMihg2yJrMswfpBxr) . Nesse documento usa-se um “Control Flow Automaton” como  modelo do programa imperativo que calcula a multiplicação de  inteiros positivos representados por vetores de bits.


1. Construir um FOTS, usando BitVec’s de tamanho $$n$$ , que descreva o comportamento deste autómato; para isso identifique e codifique em `z3-solver`  ou `pySMT`, as *variáveis* do modelo, o *estado inicial* , a *relação de transição* e o *estado de erro.*
2.  Usando Bounded Model Checking (BMC) verifique nesse SFOTS se $$\;$$a propriedade $$\;(x*y + z = a*b)\;$$ $$\;$$é um *invariante* do seu comportamento.
3. Sejam $$\,N,M\,$$ parâmetros do problema. Usando BMC em $$N$$ passos  no FOTS acima e adicionando   a condição  $$\,N \leq a,b \leq M\,$$ ao estado inicial, verifique a segurança do programa; nomeadamente  verifique que, com tal estado inicial, o estado de erro não é acessível.
