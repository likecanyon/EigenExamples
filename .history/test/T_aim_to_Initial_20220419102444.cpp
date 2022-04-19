#include <iostream>
#include <Eigen/Dense>
#include <vector> 
#include <stdio.h>
#include<Eigen/Geometry>
#include<Eigen/Core>
#include <math.h>

using Eigen::MatrixXd;
 using namespace std;
int main()
{

std::vector<double> Tcp_pose1(6);
Tcp_pose1[0]=-0.74303;Tcp_pose1[1]=0.110771;Tcp_pose1[2]=0.345171;Tcp_pose1[3]=2.10819;Tcp_pose1[4]=-2.29998;Tcp_pose1[5]=-0.0647837;
Eigen::Vector3d Translation1={Tcp_pose1[0],Tcp_pose1[1],Tcp_pose1[2]};
Eigen::Vector3d axis1={Tcp_pose1[3],Tcp_pose1[4],Tcp_pose1[5]};
Eigen::Matrix3d R_end_to_base1;
Eigen::Matrix4d T_end_to_base1;
T_end_to_base1.setIdentity();
axis1.normalize();
double alpha1=Tcp_pose1[3]/axis1[0];
R_end_to_base1=Eigen::AngleAxisd(alpha1,axis1);
T_end_to_base1.block<3,3>(0,0)=R_end_to_base1;
T_end_to_base1.block<3,1>(0,3)=Translation1;
std::cout<<"T_end_to_base1 is "<<std::endl<<T_end_to_base1<<std::endl;

std::vector<double> Tcp_pose2(6);
Tcp_pose2[0]=-0.710826;Tcp_pose2[1]=0.264062;Tcp_pose2[2]=0.217196;Tcp_pose2[3]=2.12514;Tcp_pose2[4]=-2.26665;Tcp_pose2[5]=-0.0436107;
Eigen::Vector3d Translation2={Tcp_pose2[0],Tcp_pose2[1],Tcp_pose2[2]};
Eigen::Vector3d axis2={Tcp_pose2[3],Tcp_pose2[4],Tcp_pose2[5]};
Eigen::Matrix3d R_end_to_base2;
Eigen::Matrix4d T_end_to_base2;
T_end_to_base2.setIdentity();
axis1.normalize();
double alpha2=Tcp_pose2[3]/axis2[0];
R_end_to_base2=Eigen::AngleAxisd(alpha2,axis2);
T_end_to_base2.block<3,3>(0,0)=R_end_to_base2;
T_end_to_base2.block<3,1>(0,3)=Translation2;
std::cout<<"T_end_to_base2 is "<<std::endl<<T_end_to_base2<<std::endl;



Eigen::Matrix4d T_aim_to_Initial;
T_aim_to_Initial=T_end_to_base1.inverse()*T_end_to_base2;
std::cout<<"T_aim_to_Initial is "<<std::endl<<T_aim_to_Initial<<std::endl;

std::cout<<endl;
}