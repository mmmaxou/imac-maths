// ================================
// MATHS C++ - IMAC 2
// TP 5
// ================================

#include <iostream>
#include <Eigen/Dense>
#include "math.h"
#include "assert.h"
#include <ctime>

using namespace Eigen;
using namespace std;

namespace TP_CPP_IMAC2
{
  double evalPolynomial(const VectorXd &p,  const double &x) {
    double sum = 0;
    double currentX = 1;
    for(int i=0; i<p.size(); i++) {
      sum += currentX * p(i);
      currentX *= x;
    }
    return sum;
  }
  
  VectorXd polynomialFromRoots(const VectorXd &roots) {
    // On gènere les éléments qui se suivent
    VectorXd polynome = VectorXd::Zero(roots.size()+1);
    
    // 1e Polynome
    polynome(0) = -roots(0);
    polynome(1) = 1;
    
    // Boucle
    for(int i=1; i<roots.size(); i++)
      for(int j=polynome.size()-1; j>=0; j--)
        if (j==0)
          polynome(j)=polynome(j)*(-roots(i));
        else 
          polynome(j)=polynome(j-1)+(polynome(j)*(-roots(i)));
    return polynome;
  }
  
  int main(int argc, char *argv[])
  {
    VectorXd polynome(5);
    polynome << 3.12, 4, 5.5, 1, 2.1;
    double x = 2.3;    
    cout << polynome << "\n=>\n" << evalPolynomial(polynome, x) << endl;
    
    
    cout << endl << "ROOTS" << endl << endl;
    VectorXd roots(3);
    roots << 2, 3, 4;
    cout << roots << "\n=>\n" << polynomialFromRoots(roots) << endl;
    
    return 0;
  }
}

// Fonction main classique, point d'entrée du programme
int main(int argc, char *argv[])
{
  // Appel de la fonction main de l'espace de nom TP_CPP_IMAC2
  return TP_CPP_IMAC2::main(argc, argv);
}

