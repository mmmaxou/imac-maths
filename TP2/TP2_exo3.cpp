// ================================
// MATHS C++ - IMAC 2
// TP 2 - Exercice 3
// ================================

#include <iostream>
#include <Eigen/Dense>
#include "math.h"
#include "assert.h"

// g++ -Wall -O2 -I /usr/include/eigen3 TP2_exo3.cpp -o TP2_exo3 && ./TP2_exo3

namespace TP_CPP_IMAC2
{
  /*
  Complexité en O( n ),
  n = size()
  */
  double dotProduct(const Eigen::VectorXf &v1, const Eigen::VectorXf &v2) {
    assert(v1.size() == v2.size());
    double sum = 0.0;
    for(unsigned int i=0; i<v1.size(); i++)
      sum += (v1[i] * v2[i]);
    return sum;
  }
  
  int main(int argc, char *argv[])
  {
    int dimension = 500000;
    srand((unsigned int) time(0));
    Eigen::VectorXf v1 = Eigen::VectorXf::Random(dimension);
    Eigen::VectorXf v2 = Eigen::VectorXf::Random(dimension);
//    std::cout << "v1 : " << v1.transpose() << std::endl;
//    std::cout << "v2 : " << v2.transpose() << std::endl;
    std::cout << "Dot product : " << dotProduct(v1, v2) << std::endl;
    std::cout << "Dot product : " << v1.dot(v2) << std::endl;
    return 0;
  }
}

// Fonction main classique, point d'entrée du programme
int main(int argc, char *argv[])
{
  // Appel de la fonction main de l'espace de nom TP_CPP_IMAC2
  return TP_CPP_IMAC2::main(argc, argv);
}

