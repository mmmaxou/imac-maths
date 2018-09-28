// ================================
// MATHS C++ - IMAC 2
// TP 1 - Exercice 6
// ================================

#include "./chrono.hpp"
#include <iostream>
#include "math.h"

namespace TP_CPP_IMAC2
{
  void swap(int &a, int &b) {
    int tmp = a;
    a = b;
    b = tmp;
  }
  
  int russianCalc(int a, int b)  {
    // Si a ou b nul, return 
    if ( a==0 || b==0 ) return 0;
    
    bool isNeg = false;
    if ( a<0 ) {
      isNeg = !isNeg;
      a = -a;
    }
    if ( b<0 ) {
      isNeg = !isNeg;
      b = -b;
    }
    
    // Si a est supérieur a b, on échange a et b
    if (a>b) swap(a,b);
    std::cout << "Min:" << a << " & Max:" << b << std::endl;
    
    int residu = 0;   
    
    while (a!=1) {
      residu += a&1 ? b : 0; // a Impair ==> On augmente le residu
      a = a>>1; // On divise par 2
      b = b<<1; // On multiplie par 2
      std::cout << "Min:" << a << " & Max:" << b << std::endl;
    }
    b+=residu;
    return isNeg ? -b : b;
  }
  
  int main(int argc, char *argv[])
  {
    int a=0, b=0;
    std::cout << "Entier A: ";
    std::cin >> a;
    std::cout << "Entier B: ";
    std::cin >> b;
        
    std::cout << "Résultat : " << russianCalc(a, b) << std::endl;
    
    return 0;
  }
}

// Fonction main classique, point d'entrée du programme
int main(int argc, char *argv[])
{
  // Appel de la fonction main de l'espace de nom TP_CPP_IMAC2
  return TP_CPP_IMAC2::main(argc, argv);
}

