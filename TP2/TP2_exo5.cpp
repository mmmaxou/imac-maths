// ================================
// MATHS C++ - IMAC 2
// TP 2 - Exercice 5
// ================================

#include <iostream>
#include <Eigen/Dense>
#include "math.h"
#include "assert.h"
#include <ctime>

// g++ -Wall -O2 -I /usr/include/eigen3 TP2_exo5.cpp -o TP2_exo5 && ./TP2_exo5
// g++ -Wall -O2 -I /usr/include/eigen3 TP2_exo5.cpp -o TP2_exo5 -fopenmp && ./TP2_exo5

namespace TP_CPP_IMAC2
{
  /*
    Complexité en O ( n*n*n )
  */
  Eigen::MatrixXf matrixProduct(const Eigen::MatrixXf &m1,
                                const Eigen::MatrixXf &m2) 
  {
    assert(m1.cols() == m2.rows() && "Error: Columns and Rows are different");
    
    Eigen::MatrixXf nmx = Eigen::MatrixXf::Zero(m1.rows(), m2.cols());
    
    for (unsigned int i=0; i<m1.rows(); i++)
      for (unsigned int j=0; j<m2.cols(); j++)
        for (unsigned int k=0; k<m1.cols(); k++)
          nmx(i, j) += m1(i, k) * m2(k, j);
    
    return nmx;
  }
  
  
  
  Eigen::MatrixXf strassen_recurr(const Eigen::MatrixXf &m1, 
                                  const Eigen::MatrixXf &m2) 
  {
    if ( m1.rows() <= 128 ) return m1 * m2;
    
    /* Condition d'arret */
    int rows = m1.rows()/2;
    int cols = m1.cols()/2;
    
    /* On veut séparer m1 et m2 en petits bouts de matrices */
    Eigen::MatrixXf A = m1.topLeftCorner(rows, cols);
    Eigen::MatrixXf B = m1.topRightCorner(rows, cols);
    Eigen::MatrixXf C = m1.bottomLeftCorner(rows, cols);
    Eigen::MatrixXf D = m1.bottomRightCorner(rows, cols);
    Eigen::MatrixXf E = m2.topLeftCorner(rows, cols);
    Eigen::MatrixXf F = m2.topRightCorner(rows, cols);
    Eigen::MatrixXf G = m2.bottomLeftCorner(rows, cols);
    Eigen::MatrixXf H = m2.bottomRightCorner(rows, cols);
    
    /* On calcule les coefficients P1 à P7 */
    Eigen::MatrixXf P1 = strassen_recurr(A, (F - H));
    Eigen::MatrixXf P2 = strassen_recurr((A + B), H);
    Eigen::MatrixXf P3 = strassen_recurr((C + D), E);
    Eigen::MatrixXf P4 = strassen_recurr(D, (G - E));
    Eigen::MatrixXf P5 = strassen_recurr((A + D), (E + H));
    Eigen::MatrixXf P6 = strassen_recurr((B - D), (G + H));
    Eigen::MatrixXf P7 = strassen_recurr((A - C), (E + F));
    
    /* On a notre matrice resultat */
    Eigen::MatrixXf nmx(m1.rows(), m2.cols());
    
    /* On calcule les nouvelles sous-matrices */    
    /* On rassemble des ptits bouts */    
    nmx.topLeftCorner(rows, cols) = P5 + P4 - P2 + P6;
    nmx.topRightCorner(rows, cols) = P1 + P2;
    nmx.bottomLeftCorner(rows, cols) = P3 + P4;
    nmx.bottomRightCorner(rows, cols) = P1 + P5 - P3 - P7;
    
    return nmx;
  }
  
  
  /*
    Complexité en O ( n pow 2.81 )
  */
  Eigen::MatrixXf strassen(const Eigen::MatrixXf &m1, 
                           const Eigen::MatrixXf &m2) 
  {
    assert(m1.cols() == m2.rows() && "Error: Columns and Rows are different");
    
    /* Here come STRASSEN */
    Eigen::MatrixXf nmx = strassen_recurr(m1, m2);
    
    return nmx;
  }
  
  int main(int argc, char *argv[])
  {
    int dimension = 4096;
    srand((unsigned int) time(0));
    Eigen::MatrixXf m1 = Eigen::MatrixXf::Random(dimension, dimension);
    Eigen::MatrixXf m2 = Eigen::MatrixXf::Random(dimension, dimension);
    Eigen::MatrixXf mOurs(dimension, dimension);
    Eigen::MatrixXf mStrassen(dimension, dimension);
    Eigen::MatrixXf mEigen(dimension, dimension);
    Eigen::MatrixXf mEigenNThread(dimension, dimension);
    
    clock_t begin, end;
    double tempsCalc;
    
    /* Ours */
//    begin = clock();
//    mOurs = matrixProduct(m1, m2);
//    end = clock();
//    tempsCalc = double(end - begin) / CLOCKS_PER_SEC;
//    std::cout << "Time (ours): " << tempsCalc << std::endl;
    
    
    /* Strassen */    
    begin = clock();
    mStrassen = strassen(m1, m2);
    end = clock();
    tempsCalc = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Time (strassen): " << tempsCalc << std::endl;
    
    
    /* EIGEN */
    begin = clock();
    mEigen = m1 * m2;
    end = clock();
    tempsCalc = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Time (EIGEN) : " << tempsCalc << std::endl;
      
    /* (EIGEN N-THREADED) */    
    Eigen::setNbThreads(4); // J'ai 4 coeurs
    begin = clock();
    mEigenNThread = m1 * m2;
    end = clock();
    tempsCalc = (double(end - begin) / CLOCKS_PER_SEC) / 4;
    std::cout << "Time (EIGEN N-THREADED) : " << tempsCalc << std::endl;
    
    
    
    /* Verif */    
//    std::cout << "Verification Ours -> Eigen(norm) = " << (mOurs - mEigen).norm() << std::endl;
    std::cout << "Verification Strassen -> Eigen(norm) = " << (mStrassen - mEigen).norm() << std::endl;
    std::cout << "Verification EigenNThread -> Eigen(norm) = " << (mEigenNThread - mEigen).norm() << std::endl;
    
    /*
      On optimise en utilisant :
      SMID
      Transfert cache
    */
    
    return 0;
  }
}

// Fonction main classique, point d'entrée du programme
int main(int argc, char *argv[])
{
  // Appel de la fonction main de l'espace de nom TP_CPP_IMAC2
  return TP_CPP_IMAC2::main(argc, argv);
}

