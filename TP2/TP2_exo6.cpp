// ================================
// MATHS C++ - IMAC 2
// TP 2 - Exercice 6
// ================================

#include <iostream>
#include <Eigen/Dense>
#include "math.h"
#include "assert.h"
#include <ctime>

// g++ -Wall -O2 -I /usr/include/eigen3 TP2_exo6.cpp -o TP2_exo6 && ./TP2_exo6

namespace TP_CPP_IMAC2
{  
  int main(int argc, char *argv[])
  {
    Eigen::Matrix4i permRow;
    Eigen::Matrix4i permCol;
    Eigen::Matrix4i random;
    permRow <<1, 0, 0, 0,
              0, 0, 0, 1,
              0, 0, 1, 0,
              0, 1, 0, 0;
    
    permCol <<1, 0, 0, 0,
              0, 0, 0, 1,
              0, 0, 1, 0,
              0, 1, 0, 0;
    
    random << 2, 1, 2, 2,
              -4, -3, -4, -4,
              5, 4, 5, 5,
              3, 2, 3, 3;
        
    std::cout << random << std::endl << std::endl;
    std::cout << (permRow*random) << std::endl << std::endl;
    std::cout << (random*permCol) << std::endl;
    return 0;
  }
}

// Fonction main classique, point d'entrée du programme
int main(int argc, char *argv[])
{
  // Appel de la fonction main de l'espace de nom TP_CPP_IMAC2
  return TP_CPP_IMAC2::main(argc, argv);
}

