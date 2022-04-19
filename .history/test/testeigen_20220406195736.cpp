#include <iostream>
#include <Eigen/Dense>
 
using Eigen::MatrixXd;
 
int main()
{
  MatrixXd m(3,2);
  m(0,0) = 1;
  m(1,0) = 2;
  m(2,0) = 3;
  m(0,1) = 4;
  m(1,1)=5;
  m(2,1)=6;


  std::cout << m << std::endl;
}