// ================================
// MATHS C++ - IMAC 2
// TP 2 - Exercice 4
// ================================

#include <iostream>
#include <Eigen/Dense>
#include "math.h"
#include "assert.h"

// g++ -Wall -O2 -I /usr/include/eigen3 TP2_exo4.cpp -o TP2_exo4 && ./TP2_exo4

namespace TP_CPP_IMAC2
{
  
  /*
    Complexité en O ( n*n*n )
  */
  Eigen::MatrixXf matrixProduct(const Eigen::MatrixXf &m1, const Eigen::MatrixXf &m2) {
    assert(m1.cols() == m2.rows() && "Error: Columns and Rows are different");
    
    Eigen::MatrixXf nmx = Eigen::MatrixXf::Zero(m1.rows(), m2.cols());
    
    for (unsigned int i=0; i<m1.rows(); i++)
      for (unsigned int j=0; j<m2.cols(); j++)
        for (unsigned int k=0; k<m1.cols(); k++)
          nmx(i, j) += m1(i, k) * m2(k, j);
    
    return nmx;
  }
  
  /*
    Complexité en O ( n*n*n )
  */
  Eigen::MatrixXf matrixProductDot(const Eigen::MatrixXf &m1, const Eigen::MatrixXf &m2) {
    assert(m1.cols() == m2.rows() && "Error: Columns and Rows are different");
    
    Eigen::MatrixXf nmx = Eigen::MatrixXf::Zero(m1.rows(), m2.cols());
    
    for (unsigned int i=0; i<m1.rows(); i++)
      for (unsigned int j=0; j<m2.cols(); j++)
        nmx(i, j) += m1.row(i).dot(m2.col(j));
    
    return nmx;
  }
  
  int main(int argc, char *argv[])
  {
    int dimension = 500;
    int otherDim = 400;
    srand((unsigned int) time(0));
    Eigen::MatrixXf m1 = Eigen::MatrixXf::Random(otherDim, dimension);
    Eigen::MatrixXf m2 = Eigen::MatrixXf::Random(dimension, otherDim);
    std::cout << "M1 : " << std::endl << m1 << std::endl;
    std::cout << "M2 : " << std::endl << m2 << std::endl;
    std::cout << "Matrix product : " << std::endl << matrixProduct(m1, m2) << std::endl;
    std::cout << "Matrix product : " << std::endl << matrixProductDot(m1, m2) << std::endl;
    std::cout << "Matrix product : " << std::endl << m1 * m2 << std::endl;
    std::cout << "Epsilon : " << (matrixProduct(m1, m2) - (m1*m2)).norm() << std::endl;
    return 0;
  }
}

// Fonction main classique, point d'entrée du programme
int main(int argc, char *argv[])
{
  // Appel de la fonction main de l'espace de nom TP_CPP_IMAC2
  return TP_CPP_IMAC2::main(argc, argv);
}

