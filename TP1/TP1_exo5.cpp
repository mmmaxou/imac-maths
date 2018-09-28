// ================================
// MATHS C++ - IMAC 2
// TP 1 - Exercice 5
// ================================

#include "./chrono.hpp"
#include <iostream>
#include "math.h"

namespace TP_CPP_IMAC2
{
  int main(int argc, char *argv[])
  {
    float a = powf(2.0,40);
    std::cout << " a = " << a << std::endl;
    
    int b = a;
    std::cout << " b = " << b << std::endl;
    
    int cpt = 0;
    for(double a=0.0; a<100000000; a+=0.00000001) {
      cpt++;
      if(cpt%100000==0) {
        std::cout << a << std::endl;
      } 
    }
    std::cout << " `a table ! "  << std::endl;
    
    return 0;
  }
}

// Fonction main classique, point d'entrée du programme
int main(int argc, char *argv[])
{
  // Appel de la fonction main de l'espace de nom TP_CPP_IMAC2
  return TP_CPP_IMAC2::main(argc, argv);
}

