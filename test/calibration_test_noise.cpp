#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <random>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <time.h>

// this is an example in how to use a class in one executable using full namespaces

// file that defines the class you want to use
#include "calibration/Calibration.h"

int main(int argc, char **argv) 
{

std::ofstream myfile;
myfile.open ("example.txt");
myfile << "iPos\ti_den\ti_times\t\terrorR\t\t\terrorT \n";

std::vector<double> comp_time_vec, vect1, vect2;
//int i_time = 0;

for (int i_5times = 1; i_5times <= 5; i_5times++) {

int i_pos = 1280;

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

 	Eigen::Matrix4f X = Eigen::Matrix4f::Identity();;
	Eigen::Vector4f qx_use = Eigen::Vector4f(dis(gen), dis(gen), dis(gen), dis(gen) );
	Eigen::Quaternionf qx( qx_use[0], qx_use[1], qx_use[2], qx_use[3] );
	qx.normalize();
	Eigen::Vector3f tx = Eigen::Vector3f(dis(gen),dis(gen), dis(gen));
	X.block<3,3>(0,0) = qx.toRotationMatrix();
	X.block<3,1>(0,3) = tx;
	//std::cout << "X: " << std::endl << X << std::endl;

 	int NPoses = i_pos;

	std::vector<Matrix4f> B, A;

	Eigen::Matrix4f a = Eigen::Matrix4f::Identity();


	Eigen::Vector4f q_use = Eigen::Vector4f::Random();
	Eigen::Quaternionf q( q_use[0], q_use[1], q_use[2], q_use[3]);
	q.normalize();

	Eigen::Vector3f t = Eigen::Vector3f( dis(gen), dis(gen), dis(gen) );

	a.block<3,3>(0,0) = q.toRotationMatrix();
	a.block<3,1>(0,3) = t;

	int den_test = 1024; // grid resolution

	for (int i = 0; i < NPoses; i++)
	{

		// test data
		Eigen::Matrix4f b = Eigen::Matrix4f::Identity();
		b = Z.inverse()*a*X;

		Eigen::Matrix4f d = Eigen::Matrix4f::Identity();
        
		Eigen::Vector3f t = 0.1*(1/den_test)*Eigen::Vector3f( dis(gen),dis(gen), dis(gen) );

		float roll = dis(gen)*M_PI/den_test;
		float pitch = dis(gen)*M_PI/den_test;
		float yaw = dis(gen)*M_PI/den_test;
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

	//i_den it's the sampling fraction: pick one champion every i_den
	for (int i_den = 1; i_den <= 32; i_den *= 2) {

	std::cout << std::endl << "i_pos: " << i_pos << ", " << "i_den: " << i_den << ", " << std::endl << "i_5times: " << i_5times << std::endl;

	std::vector<Matrix4f> Asampled, Bsampled; //A, B sampled
	Asampled.clear();
	Bsampled.clear();

	//fulfilling matrices A,B with choosen champions, limit per matrix sampled is NPoses (total available) over the sampling fraction i_den

	for (int i_sam = 0; i_sam < NPoses/i_den; i_sam +=i_den){
		Asampled.push_back(A[i_sam]);
		Bsampled.push_back(B[i_sam]);
	}

	// first, create an object of your class 
	calibration::Calibration mycalib;

	mycalib.setInput(Asampled, Bsampled);
	
	clock_t t1,t2;
    t1=clock();
    vect1.push_back((float(t1)*1000)/CLOCKS_PER_SEC);

	mycalib.computeClosedForm();
	
	t2=clock();
	vect2.push_back((float(t2)*1000)/CLOCKS_PER_SEC);
    float diff ((float)t2-(float)t1);
    comp_time_vec.push_back( (diff*10000) / CLOCKS_PER_SEC );
    //i_time++;
	//sleep(2);

	mycalib.computeNonLinOpt();

	Eigen::Matrix4f Tx, Tz;
	float Er, Et;
	Tx = Eigen::Matrix4f::Identity();
	Tz = Eigen::Matrix4f::Identity();
	mycalib.getFullOutput(Tz, Tx, Er, Et);

	myfile << i_pos << "\t\t" << i_den << "\t\t" << i_5times << "\t\t" << Er << "\t\t" <<  Et << "\t\t" << "\n";

	// print out the result

	std::cout << "combo: " << i_pos << " / " << i_den << " / " << i_5times << std::endl;
	
	std::cout << "Input Z: " << std::endl << Z << std::endl;
	std::cout << "Estimated Z:" << std::endl << Tz << std::endl;
	std::cout << "Error over Z estimation: " << std::endl << Z-Tz << std::endl << std::endl;
	
	std::cout << "Input X: " << std::endl << X << std::endl;
	std::cout << "Estimated X:" << std::endl << Tx << std::endl;
	std::cout << "Error over X estimation: " << std::endl << X-Tx << std::endl;

// compute Rotation and translation errors over Non Linear results

	/*Eigen::Matrix3f RxOK_NL, RzOK_NL;
	Eigen::Vector3f txOK_NL, tzOK_NL;

	RxOK_NL = Tx.block<3,3>(0,0);
	RzOK_NL = Tz.block<3,3>(0,0);
	txOK_NL = Tx.block<3,1>(0,3);
	tzOK_NL = Tz.block<3,1>(0,3);

	float Er_NL = 0;

	float EtNum_NL = 0, EtDen_NL = 1;

	for (int i = 0; i < NPoses; i++)
	{
		

		Er_NL += Matrix3f( A[i].block<3,3>(0,0)*RxOK_NL - RzOK_NL*B[i].block<3,3>(0,0) ).squaredNorm();

		EtDen_NL += Vector3f( A[i].block<3,3>(0,0)*txOK_NL - A[i].block<3,1>(0,3) ).squaredNorm();
				  
		EtNum_NL += Vector3f( A[i].block<3,3>(0,0)*txOK_NL + A[i].block<3,1>(0,3) - RzOK_NL*B[i].block<3,1>(0,3) - tzOK_NL ).squaredNorm();

	}

	float Et_NL = sqrt(EtNum_NL/EtDen_NL);

	std::cout << "Rotation error on Non Linear form is " << Er_NL << std::endl;
	std::cout << "Translation error on Non Linear form is " << Et_NL << std::endl;
	std::cout << "combo " << NPoses << "/ " << den_test << "/ " << i_5times << std::endl;*/




}
}
// chiusura dei tre for
std::cout << "closed form time computation in microseconds :"<< std::endl;
  for (unsigned i=0; i<comp_time_vec.size(); i++)
    std::cout << "i " << i<< " diff " << comp_time_vec[i] << " t1 " << vect1[i] <<" t2 " << vect2[i] << std::endl;
  std::cout << '\n';
//std::cout<< "closed form time computation in milliseconds " << std::endl << comp_time_vec << std::endl;

myfile.close();

return 0;

}