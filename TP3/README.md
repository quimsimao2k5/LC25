# Trabalho Prático 3


## Exercício 1

O [algoritmo estendido de Euclides](https://en.wikipedia.org/wiki/Extended_Euclidean_algorithm) (EXA) aceita dois inteiros constantes  $$\,a,b>0\,$$  e devolve inteiros $$r,s,t\,$$ tais que  $$\,a*s + b*t = r\,$$  e  $$\,r = \gcd(a,b)\,$$. 
Para além das variáveis $$\,r,s,t\,$$ o código requer 3 variáveis adicionais $$\,r',s',t'\,$$ que representam os valores de $$\,r,s,t\,$$ no “próximo estado”.

    
    INPUT  a, b
    assume  a > 0 and b > 0
    r, r', s, s', t, t' = a, b, 1, 0, 0, 1
    while r' != 0
      q = r div r'
      r, r', s, s', t, t' = r', r − q × r', s', s − q × s', t', t − q × t' 
    OUTPUT r, s, t
    


    1. Construa um SFOTS usando BitVector’s de tamanho $$n=16\,$$ bits que descreva o comportamento deste programa.  Considere estado de erro quando $$\,r=0\,$$ ou alguma das variáveis atinge o “overflow”.
    2. Usando a metodologia das  “Constraint Horn Clauses”(chc’s) verifique se é possível determinar um invariante que garanta que nunca se atinge um estado de erro.
    3. Verifique, usando a metodologia dos invariantes e interpolantes, se o modelo atinge um estado de erro. 
        Para o cálculo do interpolante usar a metodologia das “Constraint Horn Clauses”(chc’s).



## Exercício 2

Na continuação do problema 1 pretende-se provar a correção do programa aì apresentado.

    1. Identifique um CFA que representa o programa. Nomeadamente identifique 
        1. os locais e os transformadores de predicados “weakest pre-condition” que descrevem as transições de estado em cada local. 
        2. as guardas que determinam as transições de local
        3. os locais que representam as situações de erro e os que representam a terminação com sucesso.
    2. Usando $$k$$-indução verifique que $$\,\phi(a,b,r,s,t) \,\equiv\; a*s + b*t = r\;$$  é invariante.
    3. Usando a metodologia dos “look-aheads” verifique que o programa termina sempre.
