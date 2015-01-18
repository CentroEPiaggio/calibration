#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <random>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>

#include <gtest/gtest.h>
#include "calibration/Calibration.h"

int main(int argc, char **argv) 
{
	std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1, 1);

	// AX = ZB
	Eigen::Matrix4f Z = Eigen::Matrix4f::Identity();
	Eigen::Vector4f qz_use = Eigen::Vector4f(dis(gen), dis(gen), dis(gen), dis(gen) );
	Eigen::Quaternionf qz( qz_use[0], qz_use[1], qz_use[2], qz_use[3] );
	qz.normalize();
	Eigen::Vector3f tz = Eigen::Vector3f( dis(gen), dis(gen), dis(gen) );
	Z.block<3,3>(0,0) = qz.toRotationMatrix();
	Z.block<3,1>(0,3) = tz;
	//std::cout << "Z: " << std::endl << Z << std::endl;

 	Eigen::Matrix4f X = Eigen::Matrix4f::Identity();
	Eigen::Vector4f qx_use = Eigen::Vector4f(dis(gen), dis(gen), dis(gen), dis(gen) );
	Eigen::Quaternionf qx( qx_use[0], qx_use[1], qx_use[2], qx_use[3] );
	qx.normalize();
	Eigen::Vector3f tx = Eigen::Vector3f(dis(gen),dis(gen), dis(gen));
	X.block<3,3>(0,0) = qx.toRotationMatrix();
	X.block<3,1>(0,3) = tx;
	//std::cout << "X: " << std::endl << X << std::endl;

 	int NPoses = 6;


	std::vector<Matrix4f> A, B;

	Eigen::Matrix4f a = Eigen::Matrix4f::Identity();


	Eigen::Vector4f q_use = Eigen::Vector4f::Random();
	Eigen::Quaternionf q( q_use[0], q_use[1], q_use[2], q_use[3]);
	q.normalize();

	Eigen::Vector3f t = Eigen::Vector3f( dis(gen), dis(gen), dis(gen) );

	a.block<3,3>(0,0) = q.toRotationMatrix();
	a.block<3,1>(0,3) = t;

	for (int i = 0; i < NPoses; i++)
	
	{

		// test data
		Eigen::Matrix4f b = Eigen::Matrix4f::Identity();
		b = Z.inverse()*a*X;

		Eigen::Matrix4f d = Eigen::Matrix4f::Identity();
        
		Eigen::Vector3f t = 0.001*Eigen::Vector3f( dis(gen),dis(gen), dis(gen) );

		float roll = dis(gen)*M_PI/512;
		float pitch = dis(gen)*M_PI/512;
		float yaw = dis(gen)*M_PI/512;
		Eigen::Matrix3f R;
		R = AngleAxisf(roll, Vector3f::UnitX())
			*AngleAxisf(pitch, Vector3f::UnitY())
  			*AngleAxisf(yaw, Vector3f::UnitZ());
		d.block<3,3>(0,0) = R;
		d.block<3,1>(0,3) = t;

		a = d*a;

		A.push_back(a);
		B.push_back(b);

	}

	// first, create an object of your class 
	calibration::Calibration mycalib;

	mycalib.setInput(A, B);

	mycalib.computeNonLinOpt();

	Eigen::Matrix4f Tx, Tz;
	Tx = Eigen::Matrix4f::Identity();
	Tz = Eigen::Matrix4f::Identity();
	mycalib.getOutput(Tz, Tx);

	// print out the result


	std::cout << "Input Z: " << std::endl << Z << std::endl;
	std::cout << "Estimated Z:" << std::endl << Tz << std::endl;
	std::cout << "Error over Z estimation: " << std::endl << Z-Tz << std::endl << std::endl;

	
	std::cout << "Input X: " << std::endl << X << std::endl;
	std::cout << "Estimated X:" << std::endl << Tx << std::endl;
	std::cout << "Error over X estimation: " << std::endl << X-Tx << std::endl;

	return 0;
}