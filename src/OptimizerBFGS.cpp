#include "OptimizerBFGS.h"
#include "Objective.h"
#include <iostream>

using namespace std;
using namespace Eigen;

OptimizerBFGS::OptimizerBFGS()
{
	
}

OptimizerBFGS::~OptimizerBFGS()
{
	
}

VectorXd OptimizerBFGS::optimize(const shared_ptr<Objective> objective, const VectorXd &xInit)
{	
	auto numRows = xInit.size();
	MatrixXd I(numRows,numRows);
	//makes nxn identity matrix
	for (int i = 0; i < numRows; i++)
	{
		I(i,i) = 1;
	}
	
	MatrixXd A;
	VectorXd x(xInit);
	VectorXd x_0(numRows);
	VectorXd grad_0(numRows);
	VectorXd grad(numRows);
	VectorXd delt_x;
	double alpha;

	A = I;
	
	for(this->iter = 1; this->iter <= this->iterMax; ++(this->iter)){
		VectorXd p;
		double obj_val = objective->evalObjective(x,grad);

		if(iter > 1){
			VectorXd s = x - x_0;
			VectorXd y = grad - grad_0;

			double rho = 1.0/(y.transpose() * s);
			A = ( I - rho*s*y.transpose() ) * A * (I - rho*y*s.transpose()) + rho * s * s.transpose();
		}
		 p = -A * grad;

		 //Line search part of optimizer 
		alpha = this->alphaInit;
		for((this->iterLS) = 1; this->iterLS <= this->iterMaxLS; ++(this->iterLS)){
			delt_x = alpha * p;
			double f1 = objective->evalObjective((x + delt_x));
			if(f1 < obj_val){
				break;
			}
			alpha *= this->gamma;
		}

		//Step part of the optimizer
		x_0 = x;
		grad_0 = grad;
		x += delt_x;
		if(grad.norm() < this->tol){
			break;
		}
	}

	return x;
}
