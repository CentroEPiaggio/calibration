#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>

// this file implements the class defined in:
#include "calibration/Calibration.h"

using namespace Eigen;
using namespace calibration;
// constructor
Calibration::Calibration()
{
	RzOK_ = Matrix3f::Identity();
	RxOK_ = Matrix3f::Identity();
	TzOK_ = Matrix4f::Identity();
	TxOK_ = Matrix4f::Identity();
}

// destructor
Calibration::~Calibration()
{
	// empty destructor
}

// set functions
void Calibration::setInput(const std::vector<Matrix4f> Ta, const std::vector<Matrix4f> Tb)
{

	// set poses number from transformations size
	// check the transformations have the same poses number

	int na = Ta.size();
	int nb = Tb.size();

	if ( (na != nb) )
	{
		std::cerr << "rotation matrix dimension not compatible!! " << std::endl;
		return;
	}

	// n_poses_ is the actual number of poses recorded by camera
	n_poses_ = (na%2 == 0)? na : na - 1;


	// std::cout << "Calibrating for " << n_poses_ << " poses..." << std::endl;

	//resizeAllParametres();
	//std::cout << "all matrices resized " << std::endl;

	// push rotations and translations in respective vectors

	Ra_.clear();
	Rb_.clear();
	ta_.clear();
	tb_.clear();

	for (int i = 0 ; i < n_poses_  ;i++) 
	{ 
		Ra_.push_back( Ta[i].block<3,3>(0,0) );

		// std::cout << "Ra_ pose  " << i << std::endl << Ra_[i] << std::endl;

		Rb_.push_back( Tb[i].block<3,3>(0,0) );

		// std::cout << "Rb_ pose  " << i << std::endl << Rb_[i] << std::endl;

		ta_.push_back( Ta[i].block<3,1>(0,3) );

		// std::cout << "ta_ pose  " << i << std::endl << ta_[i] << std::endl;

		tb_.push_back( Tb[i].block<3,1>(0,3) );

		// std::cout << "tb_ pose  " << i << std::endl << tb_[i] << std::endl;

	}

	//std::cout << "rotation part extracted " << std::endl;
							  
}

// resize all parametres

int Calibration::resizeAllParametres()
{
	QqA_.resize(n_poses_);
	WqB_.resize(n_poses_);
	Ci_.resize(n_poses_);
	Ra_.resize(n_poses_);
	Rb_.resize(n_poses_);
	Rx_.resize(n_poses_);
	Rz_.resize(n_poses_);
	Sq_a.resize(n_poses_);
	Sq_b.resize(n_poses_);
	qa_.resize(n_poses_);
	qb_.resize(n_poses_);
	//qx_.resize(n_elem_);
	//qz_.resize(n_elem_);
	ta_.resize(n_poses_);
	tb_.resize(n_poses_);
	tx_.resize(n_poses_/2);
	tz_.resize(n_poses_/2);
	return 0;
}

// dual quaternions evaluation
/* 
	by the skewsymmetrics we can obtain the dual quaternions Q and W as
	combination of the skewsymmetric and the relative quaternion so to have
	qAi * qX = Q(qAi) qX, where "*" is vect prod, and also
	qBi * qZ = W(qBi) qZ, that can substitute qA * qX = qZ *qB as
	Q(qAi) qX - W(qBi) qZ = 0

	Q(qAi) = [  qA0         -qA'
				qA      qA0*I + S(qA)]

	W(qBi) = [  qB0         -qB'
				qB      qB0*I - S(qB)]

*/	
int Calibration::computeDualQuaternion()
{
	// compute skewsymmetric

	Matrix3f Sqa, Sqb;
	Matrix4f QqA, WqB;
	Quaternionf use;

	for (int i = 0 ; i < n_poses_  ;i++) 
	{
		Sqa << 0, -qa_[i].z(), qa_[i].y(),
			   qa_[i].z(), 0, -qa_[i].x(),
			   -qa_[i].y(), qa_[i].x(), 0;

		Sq_a.push_back(Sqa);

		//std::cout << "from quaternion qa_" << std::endl << Quaternionf(qa_[i]) << std::endl << "the skewsymmetric Sq_a is " << std::endl << Matrix4f(Sq_a[i]) << std::endl;

		Sqb <<  0, -qb_[i].z(), qb_[i].y(),
				qb_[i].z(), 0, -qb_[i].x(),
			  	-qb_[i].y(), qb_[i].x(), 0;
		
		//std::cout << "from quaternion qb_" << std::endl << Quaternionf(qb_[i]) << std::endl << "the skewsymmetric Sq_b is " << std::endl << Matrix4f(Sq_b[i]) << std::endl;

		QqA.block<3,3>(1,1) = Sqa + (qa_[i].w() * Matrix3f::Identity ());
		use = qa_[i].conjugate();
		QqA.block<1,4>(0,0) = Vector4f( use.w(), use.x(), use.y(), use.z() ).transpose();
		use = qa_[i];
		QqA.block<4,1>(0,0) = Vector4f( use.w(), use.x(), use.y(), use.z() );

		// std::cout  << "QqA" << QqA << std::endl;
		
		QqA_.push_back(QqA);


		WqB.block<3,3>(1,1) = -Sqb + (qb_[i].w() * Matrix3f::Identity ());
		use = qb_[i].conjugate();
		WqB.block<1,4>(0,0) = Vector4f( use.w(), use.x(), use.y(), use.z() ).transpose();
		use = qb_[i];
		WqB.block<4,1>(0,0) = Vector4f( use.w(), use.x(), use.y(), use.z() );

		// std::cout  << "WqB" << WqB << std::endl;

		WqB_.push_back( WqB );
	}

	return 0;
}



int Calibration::computeClosedForm()
{
	transformRotations2Quaternions();
	computeDualQuaternion();
	computeC();
	computeFinalQuaternions();
	transformQuaterions2Rotations();
	computeTranslations();
	setOutput();
	computeError();
	std::cout << "Compute closed form done!" << std::endl;
	return 0;
}

int Calibration::computeNonLinOpt()
{
	computeClosedForm();
	computeOneShot();
	setOutput();
	computeError();
	std::cout << "Compute non linear optimization done!" << std::endl;
	return 0;
}

// C evaluation

/*

C matrix is the compute by the sum of the several Ci (orthogonal matrix of rank equal to 4)
obtained as Ci = -Q (qAi)'W(qBi)

C importance is due to its bond with qz and qx, because of qz is an eigenvector of the symmetric
semipositive definite matrix C'C and by that we can obtain qx = C qz/(lambda-n_poses)

*/

int Calibration::computeC()
{
	C_ = Matrix4f::Zero();
	for ( int i=0; i < n_poses_; i++ ) 
		C_ += -QqA_[i].transpose()* WqB_[i];
	return 0;
}

// final quaternions qxOK and qzOK evaluation
// alpha and lambda evaluation by eigenvalues

/*

alpha is a 4 float vector made of the C'C eigenvalues 
lambda is n_poses,alpha combination (lambda = n_poses +/- sqrt (alpha))
so to have the minimum positive of lambda choosing the best alpha of the list

k_ is the best alpha index so to remember it also when compute the relative eigenvector

*/

int Calibration::computeFinalQuaternions()
{
	EigenSolver<Matrix4f> solver (C_.transpose()*C_ );
	alpha_ = solver.eigenvalues().real();
	k_= 0;
	float Lt;

	lambda_ = ((n_poses_ - sqrt(alpha_(k_)))>0) ? (n_poses_ - sqrt(alpha_(k_))) : (n_poses_ + sqrt(alpha_(k_)));

	for (int i=1; i < 4;i++)
	{
		Lt = n_poses_ - sqrt(alpha_[i]);

		if (Lt < lambda_ && Lt >0) 
		{
			lambda_ = Lt; 
			k_ = i; 
		}
		else
		{
			Lt = n_poses_ + sqrt(alpha_[i]);
			if (Lt < lambda_) 
			{ 
				lambda_ = Lt; 
				k_ = i; 
			}
		}
	}

	Vector4f qzOK, qxOK;
	
	qzOK = solver.eigenvectors().col(k_).real();
	qxOK = C_*qzOK/(lambda_ - n_poses_);

	qxOK.normalize();

	qzOK_ = Quaternionf( qzOK[0], qzOK[1], qzOK[2], qzOK[3] );
	qxOK_ = Quaternionf( qxOK[0], qxOK[1], qxOK[2], qxOK[3] ); 

	return 0;
}



// transform all rotations needed to quaternions

int Calibration::transformRotations2Quaternions()
{
	for (int i = 0; i < n_poses_; i++)
	{
		qa_.push_back( Quaternionf (Ra_[i]) );
		qb_.push_back( Quaternionf (Rb_[i]) );
	}
	return 0;
}

// transform all quaternions needed to rotations (qxOK and qzOK)

int Calibration::transformQuaterions2Rotations()
{
	RxOK_ = qxOK_.toRotationMatrix();
	RzOK_ = qzOK_.toRotationMatrix();
}

int Calibration::computeTranslations()
{
	if (n_poses_ == 1) 
	{
		std::cerr << "translations not computable by only one rotation!! " << std::endl;
		return 0;
	}

	txOK_ = Vector3f::Zero ();
	tzOK_ = Vector3f::Zero ();

	Eigen::MatrixXf A;
	A.resize(3*n_poses_,6);
	Eigen::VectorXf c;
	c.resize(3*n_poses_);
	Eigen::VectorXf x;
	x.resize(6);

	for (int i = 0; i < n_poses_ ; i++ )
	{
		A.block<3,3>(3*i,0) = Ra_[i];
		A.block<3,3>(3*i,3) = -1*Eigen::Matrix3f::Identity();

		c.block<3,1>(3*i,0) = RzOK_*tb_[i] - ta_[i];
	}

	MatrixXf Ainv;
	Ainv.resize(6,6);
	Ainv = A.transpose()*A;

	x = Ainv.inverse()*A.transpose()*c;

	txOK_ = x.block<3,1>(0,0);
	tzOK_ = x.block<3,1>(3,0);

	/*for (int i = 0; i < n_poses_; i += 2)
	{
		txOK_ += (Matrix3f ( Ra_[i] - Ra_[i+1]).inverse() )*( RzOK_*(tb_[i] - tb_[i+1]) + ta_[i+1] - ta_[i] );
	}

	txOK_ *= 2/n_poses_;
	
	for (int i = 0; i < n_poses_; i += 2)
	{
		tzOK_ += Ra_[i]*txOK_ + ta_[i] - RzOK_*tb_[i];
	}

	tzOK_ *= 2/n_poses_;*/


	return 0;
}

int Calibration::computeError()
{
	Er_ = 0;

	float EtNum = 0, EtDen = 1;

	for (int i = 0; i < n_poses_; i++)
	{
		Er_ += Matrix3f( Ra_[i]*RxOK_ - RzOK_*Rb_[i] ).squaredNorm();

		EtDen += Vector3f( Ra_[i]*txOK_ - ta_[i] ).squaredNorm();
				  
		EtNum += Vector3f( Ra_[i]*txOK_ + ta_[i] - RzOK_*tb_[i] - tzOK_ ).squaredNorm();

	}

	//Et_ = sqrt(EtNum/EtDen);
	Et_ = EtNum/n_poses_;
	Er_ = Er_/n_poses_;

	std::cout << "Rotation error is: " << Er_ << std::endl;
	std::cout << "Translation error is: " << Et_ << std::endl;

	return 0;
}

int Calibration::convertFromParamVect2ValuesOk(const VectorXf &in, Matrix3f &RzOK, Matrix3f &RxOK, Vector3f &tzOK, Vector3f &txOK)
{
	for (int i = 0; i < 3; i++)
	{
		RxOK.col(i) = in.block<3,1>(3*i,0);
		RzOK.col(i) = in.block<3,1>((3*i)+9,0);
	}

	txOK = in.block<3,1>(18,0);	
	tzOK = in.block<3,1>(21,0);

return 0;

}

int Calibration::convertFromValuesOk2ParamVect(const Matrix3f &RzOK, const Matrix3f &RxOK, const Vector3f &tzOK, const Vector3f &txOK, VectorXf &out)

{
	for (int i = 0; i < 3; i++)
	{
		out.block<3,1>(3*i,0) = RxOK.col(i);
		out.block<3,1>((3*i)+9,0) = RzOK.col(i);
	}

	out.block<3,1>(18,0) = txOK;	
	out.block<3,1>(21,0) = tzOK;

	//std::cout << "out: " << out << std::endl;

return 0;
	
}

void Calibration::setOutput()
{
	TzOK_.block<3,3>(0,0) = RzOK_;
	TzOK_.block<3,1>(0,3) = tzOK_;

	TxOK_.block<3,3>(0,0) = RxOK_;
	TxOK_.block<3,1>(0,3) = txOK_;

	return;
}

void Calibration::getOutput(Matrix4f &TzOK, Matrix4f &TxOK)
{
	TzOK = TzOK_;
	TxOK = TxOK_;
	return;
}

void Calibration::getFullOutput(Matrix4f &TzOK, Matrix4f &TxOK, float &Er, float &Et)
{
	getOutput(TzOK, TxOK);
	Er = Er_;
	Et = Et_;
	return;
}

int Calibration::computeOneShot()
{
	FF_.n_poses__ = n_poses_;
	FF_.Ra__ = Ra_;
	FF_.Rb__ = Rb_;
	FF_.ta__ = ta_;
	FF_.tb__ = tb_;

	Eigen::NumericalDiff<my_functor> numDiff(FF_);
	Eigen::LevenbergMarquardt<Eigen::NumericalDiff<my_functor>,float> lm_solver(numDiff);

	Eigen::VectorXf x(24);
	Eigen::VectorXf x_ini(24);

	// from initial guess to x
	convertFromValuesOk2ParamVect(RzOK_, RxOK_, tzOK_, txOK_, x);
	
	//x.setRandom();
	x_ini = x;
	// std::cout << "initial guess x : " << std::endl << x << std::endl;

	// init solver parameters
	lm_solver.parameters.maxfev = 2000;
	//lm_solver.parameters.xtol = 1.0e-10;
	//lm_solver.parameters.ftol = 1.0e-10;

	Eigen::LevenbergMarquardtSpace::Status status = lm_solver.minimize(x);

	std::cout << "Iterations in optimization before ending: " << lm_solver.iter << std::endl;
	std::cout << "lm_solver.nfev " << lm_solver.nfev << std::endl;
	std::cout << "lm_solver.njev " << lm_solver.njev << std::endl;
	std::cout << "lm_solver.fnorm " << lm_solver.fnorm << std::endl;
	std::cout << "status code: " << status << std::endl;
	// std::cout << "x that minimizes the function: " << std::endl << x << std::endl;

	std::cout << "|x_ini - x_opt|: " << std::endl << x_ini-x << std::endl;

	// from x to solution
	convertFromParamVect2ValuesOk(x, RzOK_, RxOK_, tzOK_, txOK_);

	std::cout << "RxOK_.determinant() : " << RxOK_.determinant() << std::endl;
	std::cout << "RzOK_.determinant() : " << RzOK_.determinant() << std::endl;

	return 0;
}
