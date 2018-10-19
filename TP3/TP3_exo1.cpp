// ================================
// MATHS C++ - IMAC 2
// TP 3 - Exercice 1
// ================================

#include <iostream>
#include <Eigen/Dense>
#include "math.h"
#include "assert.h"
#include <ctime>

// g++ -Wall -O2 -I /usr/include/eigen3 TP3_exo1.cpp -o TP3_exo1 && ./TP3_exo1
using namespace Eigen;
using namespace std;

namespace TP_CPP_IMAC2
{  
  
  MatrixXd jacobiGS(const MatrixXd &A, const VectorXd &B, const int nb_iter) {
    VectorXd x = B;
    for( int iter=0; iter<nb_iter; iter++) {
      for( int i=0; i<A.rows(); ++i) {
        x[i] = B[i];
        for( int j=0; j<A.cols(); ++j) {
          if (i!=j) x[i] -= A(i, j) * x[j];   
        }
        x[i] /= A(i, i);
      }
    }
    return x;
  }
  
  int main(int argc, char *argv[])
  {
    /* Génerer un systeme matriciel Ax = B aléatoire */
    const int dimension = 50;
    srand(time(0));
    MatrixXd A = MatrixXd::Random(dimension, dimension);
    A += dimension * MatrixXd::Identity(dimension, dimension);
    VectorXd B = VectorXd::Random(dimension);
    
    cout << "JacobiGS : " << (B - A*jacobiGS(A, B, 20)).norm() << endl;
    PartialPivLU<MatrixXd> lu(A);
    ColPivHouseholderQR<MatrixXd> qr(A);
    JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
    cout << "LU : " << (B - A*lu.solve(B)).norm() << endl;
    cout << "QR : " << (B - A*qr.solve(B)).norm() << endl;
    cout << "SVD : " << (B - A*svd.solve(B)).norm() << endl;
    
    return 0;
  }
}

// Fonction main classique, point d'entrée du programme
int main(int argc, char *argv[])
{
  // Appel de la fonction main de l'espace de nom TP_CPP_IMAC2
  return TP_CPP_IMAC2::main(argc, argv);
}

