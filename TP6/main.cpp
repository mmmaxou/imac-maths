// ================================
// MATHS C++ - IMAC 2
// TP 6
// ================================

#include <iostream>
#include <Eigen/Dense>
#include "math.h"
#include "assert.h"
#include <ctime>
#include <cmath>
#include <climits>
#include <random>
#include <chrono>
#include <functional> // for bind

using namespace Eigen;
using namespace std;

namespace TP_CPP_IMAC2_1
{
  int main(int argc, char *argv[])
  {
    
    // On fait des nombres entre -1 et 1
    double x, y;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    //std::default_random_engine defaultGenerator(seed);
    std::mt19937_64 generator(seed);
    int a = 1000;
    int a2 = a*a;
    //double max = 2147483647;
    double max = 214748364;
    
    std::uniform_real_distribution<float> uniformRealDistribution(0,a);
    auto binded = std::bind(uniformRealDistribution, generator);
    
    int cpt = 0;
    for (int i=0; i<max; i++) {
      x = binded();
      y = binded();
      cpt += (x*x)+(y*y) < a2 ? 1 : 0;      
    }
    
    double area = 4 * cpt / (double)max;
    
    std::cout << "PI : " << area << std::endl;
    std::cout << "DELTA PI : " << M_PI - area << std::endl;
    
    return 0;
  }
}


namespace TP_CPP_IMAC2_2
{
  int main(int argc, char *argv[])
  {
    double totSize = 10.0;
    double halfSize = totSize / 2;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937_64 generator(seed);
    std::uniform_real_distribution<float> uniformRealDistribution(0, totSize);
    auto binded = std::bind(uniformRealDistribution, generator);
    int cpt = 0;
    int N = 1000000000;
    for (int i=0; i<N; i++) {
      double cut1 = binded();
      double cut2 = binded();

      double cutMin = min(cut1, cut2);
      double cutMax = max(cut1, cut2);

      double n1 = cutMin;
      double n3 = totSize - cutMax;
      double n2 = cutMax - cutMin;

      double nMax = max(max(n1, n2), n3);
      cpt += nMax > halfSize ? 1 : 0;
    }
    
    std::cout << (1 - (double)cpt / (double)N) * 100 << "%" << std::endl;
    
    return 0;
  }
}

// Fonction main classique, point d'entrée du programme
int main(int argc, char *argv[])
{
  // Appel de la fonction main de l'espace de nom TP_CPP_IMAC2
  //return TP_CPP_IMAC2_1::main(argc, argv);
  return TP_CPP_IMAC2_2::main(argc, argv);
}



