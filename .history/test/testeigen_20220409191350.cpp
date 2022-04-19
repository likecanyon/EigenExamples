#include <iostream>
#include <Eigen/Dense>
#include <vector> 
#include <stdio.h>

using Eigen::MatrixXd;
 using namespace std;
#define PI 3.1415926535897932384626433832795

Eigen::Matrix4d translate(Eigen::Vector3d xyz)               //  平移齐次坐标变换（平移算子） 
{
	// Identity()函数为单位对角阵，由左上角到右下角对角线值为1，其余为0。 Matrix4d对应4行4列矩阵、Vector3d对应3维向量
	
	Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);

	T(0, 3) = xyz(0);     
	T(1, 3) = xyz(1);
	T(2, 3) = xyz(2);

	return T;
}


int main()
{
  Eigen::Vector3d a ={1,2,3};
  Eigen::Matrix4d T = translate(a);


  std::cout << T << std::endl;
}