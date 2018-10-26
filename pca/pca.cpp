#include <iostream>
#include <Eigen/Eigenvalues>

#include "ioEigen.hpp"

using namespace Eigen;

void hints(void){

  MatrixXd M;
  
  // compute the column wize mean of a data matrix
  VectorXd mean =  M.colwise().sum() / (double) M.rows(); 
  
  // some hints to center some data (with the exterior product)
  std::cout << VectorXd::Ones(M.rows())*mean.transpose() << std::endl;

  // compute some eigen vectors
  SelfAdjointEigenSolver<MatrixXd> eigensolver(M);
  std::cout << "\neigenvectors of M \n" << eigensolver.eigenvectors().rowwise().reverse() << std::endl;
  std::cout << "\neigenvalues of M : " << eigensolver.eigenvalues().colwise().reverse().transpose() << std::endl;
  
  // extract some rows or columns from a matrix
  std::cout << M.leftCols(3) << std::endl; 
  std::cout << M.topRows(3) << std::endl; 
	
}


int main(int argc, char **argv)
{
  if(argc != 2){
    std::cerr << "usage : " << argv[0] << " data.mat" << std::endl;
    exit(0);
  }
  
  // load the data
  MatrixXd A;
  loadMatrix(A,argv[1]);
  std::cout << "A" << std::endl << A << std::endl;
  
  // mean of the data
  VectorXd mean = A.colwise().sum() / (double) A.rows();
  std::cout << "\nmean:" << std::endl << mean << std::endl;

  // center the data
  MatrixXd centered = A - VectorXd::Ones(A.rows()) * mean.transpose();
  std::cout << "\ncentered:" << std::endl << centered << std::endl;

  // normalize the data
  MatrixXd scale = MatrixXd::Zero(A.cols(), A.cols());
  scale.diagonal() = centered.cwiseAbs().colwise().sum() / (double) centered.rows();
  scale = scale.inverse();
  MatrixXd normalized = centered * scale;
  std::cout << "\nnormalized:" << std::endl << normalized << std::endl;

  // build the covariance matrix 
  MatrixXd covariance = (normalized.transpose() * normalized) / (normalized.rows()-1);
  std::cout << "\ncovariance:" << std::endl << covariance << std::endl;
  
  // compute the eigen vectors
  SelfAdjointEigenSolver<MatrixXd> eigensolver(covariance);
  std::cout << "\neigen Vectors of covariance \n" << eigensolver.eigenvectors().rowwise().reverse() << std::endl;
  std::cout << "\nneigen Values of covariance \n" << eigensolver.eigenvalues().colwise().reverse().transpose() << std::endl;
  
  // keep only n dimensions
  const int dimension = 3;
  MatrixXd T = eigensolver.eigenvectors().rowwise().reverse().leftCols( dimension );
  MatrixXd Afinal = normalized * T;
  
  // Test
  VectorXd x(A.cols());
  //x << 0, 3, 7, 10, 1;
  x << 1, 3, 3, 4, 0.1;
  VectorXd xProj = (T.transpose() * scale * (x-mean)).transpose();
  VectorXd distance = ( Afinal - VectorXd::Ones(Afinal.rows())*xProj.transpose()).rowwise().norm();
  
  std::cout << "\nxProj:" << std::endl << xProj << std::endl;
  std::cout << "\ndistance:" << std::endl << distance << std::endl;
  
  // project the data
  // ...
  
  // project a new vector (remind to center and scale this vector)
  // ...

  return 0;
}
