#include "OptimizerGD.h"
#include "Objective.h"
#include <iostream>

using namespace std;
using namespace Eigen;

OptimizerGD::OptimizerGD()
{
	
}

OptimizerGD::~OptimizerGD()
{
	
}

VectorXd OptimizerGD::optimize(const shared_ptr<Objective> objective, const VectorXd &xInit)
{
	double obj_val;
	VectorXd grad(xInit.size());
	VectorXd xFinal(xInit);
	double alpha = alphaInit;
	VectorXd delt_x;

	for (this->iter = 0; this->iter < this->iterMax; ++(this->iter)){
		
		//gets the direction we will search in 
		obj_val = objective->evalObjective(xFinal,grad);
		VectorXd p = -grad;

		//line search part (for just grad descent can set iterMaxLS to 1 )
		alpha = alphaInit;
		for(this->iterLS = 0; this->iterLS < this->iterMaxLS; ++(this->iterLS)){
			delt_x = alpha * p;
			double f1 = objective->evalObjective(xFinal + delt_x);
			if(f1 < obj_val){
				break;
			}
			alpha *= this->gamma;
		}

		//increasing the step part
		xFinal += delt_x;
		//gets magnitude/length of gradient using this part of eigen
		//++++++++++++++++++++++++ used GPT to find this member fuction from the docs +++++++++++++++++++++++++++
		double grad_mag  = grad.norm();
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		if(grad_mag < tol){
			break;
		}

	}
	return xFinal;
}

