Este dois exercícios são apresentados como introdução à modelação de problemas através de “SMT solvers” (Z3, CVC5,…) via uma das interfaces Python (z3-solver ou pySMT ) .

Exercício 1

Neste exercício vai-se considerar a modelação de circuitos do 2º grau com falhas como estão descritos neste documento . Recomenda-se a leitura atenta desse documento antes de considerar o resto do enunciado deste exercício.

Construa uma resolução das seguintes questões a partir de “inputs” do problema: os parâmetros $$\,\kappa\,,\,n\,$$ e de probabilidade de falha $$\,\varepsilon\,$$ restrita apenas às “gates” and.

1. Construa algoritmos para, sob “inputs” do segredo $$\,z\in\{0,1\}^n\,$$ e da “chave mestra” $$\,s\in\{0,1\}^\kappa\,$$, construa o circuito. Adicionalmente a partir deste circuito , construa o modelo SMT do circuito com falhas.
2. Usando o modelo acima, tente construir uma possível estimativa para $$z$$ numa execução com falhas não nulas; isto é, encontrar
   1. $$\,z'\in \{0,1\}^n\,$$ que é raíz de todos os polinómios que formam o circuito e
   2. uma situação de falhas não nulas em “gates” and que conduz a essa estimativa.
3. Conhecido $$\,z\in\{0,1\}^n\,$$ pretende-se maximizar a probabilidade de falhas and sem que o “output” $$\,0^n\,$$ seja alterado.
