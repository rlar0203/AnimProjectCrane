#include "OptimizerNM.h"
#include "Objective.h"
#include <iostream>

using namespace std;
using namespace Eigen;

OptimizerNM::OptimizerNM()
{
	
}

OptimizerNM::~OptimizerNM()
{
	
}

VectorXd OptimizerNM::optimize(const shared_ptr<Objective> objective, const VectorXd &xInit)
{
	auto vect_rows = xInit.size();
	VectorXd grad(vect_rows);
	MatrixXd Hess(vect_rows,vect_rows);
	double alpha = alphaInit;
	VectorXd delt_x;
	VectorXd xFinal(xInit);

	for(this->iter = 0; this->iter < this->iterMax; ++(this->iter)){
		
		//gets the search direction
		double obj_val = objective->evalObjective(xFinal,grad,Hess);
		VectorXd p = -Hess.ldlt().solve(grad);
		alpha = alphaInit;

		// LINE SEARCH PART
		for(this->iterLS = 0; this->iterLS < this->iterMaxLS; ++(this->iterLS)){
			delt_x = alpha * p;
			double f1 = objective->evalObjective(xFinal+delt_x);
			if(f1 < obj_val){
				break;
			}
			alpha *= this->gamma;
		}

		//Step Part 
		xFinal += delt_x;
		if( grad.norm() < tol){
			break;
		}
	}
	return xFinal;
}
