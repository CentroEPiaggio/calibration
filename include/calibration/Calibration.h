#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
//unsupported
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
//#include <unsupported/Eigen/KroneckerProduct>
#include <Eigen/LU>

// this is to avoid writing Eigen::Matrix3f, Eigen::Vector3f
// so you can write directly Matrix3f, or Vector3f
using namespace Eigen;

// project namespace
namespace calibration
{

class Calibration
{
private:

	// final values
	Vector4f alpha_;
	Quaternionf qxOK_, qzOK_;
	Matrix4f C_;
	// dual quaternion and rotation matrix vectors
	std::vector<Matrix4f> Ci_, QqA_, WqB_;
	std::vector<Matrix3f> Ra_, Rb_, Rx_, Rz_, Sq_a, Sq_b;

	// quaternions and translation vectors
	std::vector<Quaternionf> qa_, qb_;
	std::vector<Vector3f> ta_, tb_, tx_, tz_;

	float lambda_;
	int n_poses_, n_elem_, k_;

	// results
	Matrix3f RzOK_, RxOK_;
	Vector3f tzOK_, txOK_;
	Matrix4f TzOK_, TxOK_;
	float Er_, Et_;
	
	int resizeAllParametres();

	int computeDualQuaternion();

	int computeC();

	int computeFinalQuaternions();

	int transformRotations2Quaternions();

	int transformQuaterions2Rotations();

	int computeTranslations();

	int computeError();

	int convertFromParamVect2ValuesOk(const VectorXf &in, Matrix3f &RzOK, Matrix3f &RxOK, Vector3f &tzOK, Vector3f &txOK);

	int convertFromValuesOk2ParamVect( const Matrix3f &RzOK, const Matrix3f &RxOK, const Vector3f &tzOK, const Vector3f &txOK, VectorXf &out);

	void setOutput();



public:

	// constructor/destructor
	Calibration();
	~Calibration();

	int computeNonLinOpt();
	int computeClosedForm();

	void getOutput(Matrix4f &TzOK, Matrix4f &TxOK);
	void getFullOutput(Matrix4f &TzOK, Matrix4f &TxOK, float &Er, float &Et);

	void setInput(const std::vector<Matrix4f> Ta, const std::vector<Matrix4f> Tb);

	// Generic functor
	template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
	struct Functor
	{
		typedef _Scalar Scalar;
		enum
		{
		    InputsAtCompileTime = NX,
		    ValuesAtCompileTime = NY
		};
		typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
		typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
		typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

		int m_inputs, m_values;

		Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
		Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

		int inputs() const { return m_inputs; }
		int values() const { return m_values; }

	};

	struct my_functor : Functor<float>
	{
		my_functor(void): Functor<float>(24,24) {}
		std::vector<Matrix3f> Ra__, Rb__;
		std::vector<Vector3f> ta__, tb__;
		int n_poses__;
		int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
		{
		    Matrix3f RzOK, RxOK;
			Vector3f tzOK, txOK;

			for (int i = 0; i < 3; i++)
			{
				RxOK.col(i) = x.block<3,1>(3*i,0);
				RzOK.col(i) = x.block<3,1>((3*i)+9,0);
			}

			txOK = x.block<3,1>(18,0);	
			tzOK = x.block<3,1>(21,0);

			float m1 = 1, m2 = 1, m3 = 1.0e+6, m4 = 1.0e+6;

			float phi1 = 0;
			float phi2 = 0;

		    // Implement function to minimize
			for (int i = 0; i < n_poses__; i++)
			{
				phi1 += ( Matrix3f( Ra__[i]*RxOK - RzOK*Rb__[i] ).squaredNorm() );
				phi2 += ( Vector3f( Ra__[i]*txOK + ta__[i] - RzOK*tb__[i] - tzOK ).squaredNorm() );
			}

			fvec(0) = m1*phi1;
			fvec(1) = m2*phi2;
			fvec(2) = m3*( Matrix3f( RxOK*(RxOK.transpose()) - Matrix3f::Identity() ).squaredNorm() );
			fvec(3) = m4*(RxOK.determinant() -1); 
			fvec(4) = m3*( Matrix3f( RzOK*(RzOK.transpose()) - Matrix3f::Identity() ).squaredNorm() );
			fvec(5) = m4*(RzOK.determinant() -1); 



			//fvec(0) = 10.0*pow(x(0)+3.0,2) +  pow(x(1)-5.0,2);
			for(int i = 6 ; i<24; i++)
			{
				fvec(i) = 0;
			}


			// std::cout << "fvec : " << fvec << std::endl;

		    return 0;
		}
	};

	my_functor FF_;

	int computeOneShot();

};

}