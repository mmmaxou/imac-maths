Pseudo code de la méthode naive de recherche de 0 dans une fonction
On assume que les conditions suivantes sont respéctées :
- Représente une fonction continue
- Fonction monotone
- f(a) * f(b) < 0 ( s'annule )

#### === PSEUDO-CODE ===

Input : **f, a, b, dx**

**x** = **a + dx**

Tant que ( **f(a) * f(x) > 0** )
  > **x** += **dx**

return **x-dx/2**