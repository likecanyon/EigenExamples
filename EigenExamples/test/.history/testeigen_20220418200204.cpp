#include <iostream>
#include <Eigen/Dense>
#include <vector> 
#include <stdio.h>
#include<Eigen/Geometry>
#include<Eigen/Core>
#include <math.h>

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

// 旋转
Eigen::Matrix4d rotation(char axis, double radian)           //  旋转齐次坐标变换（旋转算子）
{
	Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);

	double ct = cos(radian);
	double st = sin(radian);

	switch (axis)
	{
	case 'x':
	case 'X':
		T << 1, 0, 0, 0,
			0, ct, -st, 0,
			0, st, ct, 0,
			0, 0, 0, 1;
		break;
	case 'y':
	case 'Y':
		T << ct, 0, st, 0,
			0, 1, 0, 0,
			-st, 0, ct, 0,
			0, 0, 0, 1;
		break;
	case 'z':
	case 'Z':
		T << ct, -st, 0, 0,
			st, ct, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		break;
	}

	return T;
}


int main()
{
//   Eigen::Vector3d a ={1,2,3};
//   Eigen::Matrix4d T = translate(a);
//   std::cout << T << std::endl;
// Eigen::Matrix4d T2 = rotation ('x',PI/2);
// std::cout << T2 <<endl;

//  Eigen::Matrix3d R3 ;

//  Eigen::Quaterniond quaternion(-0.00710,0.983231,0.176474,0.0454254);


// //四元数转旋转变换矩阵
//  R3=quaternion.toRotationMatrix();

//  std::cout<<R3<<std::endl;

// //转其次坐标
// Eigen::Matrix4d T3;
// Eigen::Vector3d b ={1,2,3};
// T3.setIdentity();
// T3.block<3,3>(0,0)=R3;
// T3.block<3,1>(0,3)=b;
// std::cout<<T3<<std::endl;


// Eigen::Matrix3d rotation_matrix;
// Eigen::Vector3d euler_angles =rotation_matrix.eulerAngles(2,1,0);//旋转矩阵转成Euler ZYX

// Eigen::Vector3d rpy = rotation_matrix.eulerAngles(0,1,2);//

std::vector<double> Tcp_pose(6);
Tcp_pose[0]=-0.76081;Tcp_pose[1]=0.12072;Tcp_pose[2]=0.4138;Tcp_pose[3]=-2.14272;Tcp_pose[4]=2.28824;Tcp_pose[5]=0.001;
Eigen::Vector3d Translation={Tcp_pose[0],Tcp_pose[1],Tcp_pose[2]};
Eigen::Vector3d axis={Tcp_pose[3],Tcp_pose[4],Tcp_pose[5]};
Eigen::Matrix3d R_end_to_base;
Eigen::Matrix4d T_end_to_base;
T_end_to_base.setIdentity();
axis.normalize();
double alpha=Tcp_pose[3]/axis[0];
R_end_to_base=Eigen::AngleAxisd(alpha,axis);
T_end_to_base.block<3,3>(0,0)=R_end_to_base;
T_end_to_base.block<3,1>(0,3)=Translation;
std::cout<<T_end_to_base<<std::endl;

Eigen::AngleAxisd K_marker_Initial;
K_marker_Initial.fromRotationMatrix(R_end_to_base);
Eigen::Vector3d Khat_marker_Initial_Eigen;
Khat_marker_Initial_Eigen=K_marker_Initial.axis();
std::cout<<"Khat_marker_Initial_Eigen"<<Khat_marker_Initial_Eigen[0]<<" "<<Khat_marker_Initial_Eigen[1]<<" "<<Khat_marker_Initial_Eigen[2]<<" "<<std::endl;
double alpha_marker_Initial;
alpha_marker_Initial=K_marker_Initial.angle();
std::cout<<"角度是 "<<alpha_marker_Initial<<std::endl;
std::vector<double> Khat(3);
Khat[0]=Khat_marker_Initial_Eigen(0);Khat[1]=Khat_marker_Initial_Eigen(1);Khat[2]=Khat_marker_Initial_Eigen(2);
std::cout<<"Khat"<<std::endl;

std::vector<double> target={T_end_to_base(0,3),T_end_to_base(1,3),T_end_to_base(2,3),Khat[0]*alpha_marker_Initial,Khat[1]*alpha_marker_Initial,Khat[2]*alpha_marker_Initial};



}