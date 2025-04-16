#include "ObjectiveLink.h"
#include <cmath>

using namespace Eigen;
using namespace std;

ObjectiveLink::ObjectiveLink(Vector3d weights, int num_links,const VectorXd pos_targ, std::vector<double> _lengths){
    TargWeight = weights(0);
    this->pos_targ = pos_targ;
	lengths = _lengths;
	//gets nxn wreg matrix 
	Wreg = MatrixXd(num_links,num_links);
	for(int i = 0; i < num_links; i++) {
		if(i == 0){
			Wreg(i,i) = weights(1);
		}
		else{
			Wreg(i,i) = weights(2);
		}
	}
	//adds the homogenous coord
	this->r = VectorXd::Zero(3);
	this->r(0) = 1;
	this->r(2) = 1;
}

ObjectiveLink::~ObjectiveLink()
{

}

//this only returns the objective value
double ObjectiveLink::evalObjective(const VectorXd &x) const
{	
	double obj_val;
// Ensure pos_targ is passed correctly to CalcDeltPos
	VectorXd delta_p = CalcDeltPos(this->pos_targ, x);

	obj_val = 0.5 * TargWeight * delta_p.transpose() * delta_p; 
	obj_val += 0.5 * x.transpose() * this->Wreg * x;

	return obj_val;
}

//note that the vectors g and H are used to store the input Hessian and gradient
double ObjectiveLink::evalObjective(const VectorXd &x, VectorXd &g) const
{
	double obj_val;
// Ensure pos_targ is passed correctly to CalcDeltPos
VectorXd delta_p = CalcDeltPos(this->pos_targ, x);

	obj_val = 0.5 * TargWeight * delta_p.transpose() * delta_p; 
	obj_val += 0.5 * x.transpose() * this->Wreg * x;

	g = CalcGrad(delta_p,x);

	return obj_val;
}

double ObjectiveLink::evalObjective(const VectorXd &x, VectorXd &g, MatrixXd &H) const
{
	double obj_val;
	// Ensure pos_targ is passed correctly to CalcDeltPos
	VectorXd delta_p = CalcDeltPos(this->pos_targ, x);

	obj_val = 0.5 * TargWeight * delta_p.transpose() * delta_p; 
	obj_val += 0.5 * x.transpose() * this->Wreg * x;

	g = CalcGrad(delta_p,x);

	return obj_val;

}

VectorXd ObjectiveLink::CalcDeltPos(const Eigen::VectorXd &pos_targ, const Eigen::VectorXd &theta) const {
	VectorXd delta_p(2);
	VectorXd pos(3);
	MatrixXd T;
	MatrixXd R;

	MatrixXd mat_prod;
	mat_prod = MatrixXd::Identity(3,3);


	// aggreggates the T_i()R_i() with the target pos
	for (int i = 0; i < theta.size(); i++)
	{
		//Uses identity for transform 
		T = Eigen::MatrixXd::Identity(3,3);
		if(i > 0){
			//sets Tx and Ty if not first (T_1)
			T(0,2) = this->lengths.at(i);
			T(1,2) = 0;
		}
		R = Eigen::MatrixXd::Identity(3,3);
		R(0,0) = cos(theta(i));
		R(0,1) = -sin(theta(i));
		R(1,0) = sin(theta(i));
		R(1,1) = cos(theta(i));

		 mat_prod = mat_prod * T * R;
	}
	pos = mat_prod * r;
	//gets the 2x1 excluding the homogenous coord 
	delta_p = (pos.segment<2>(0) - pos_targ);

	return delta_p;
}

VectorXd ObjectiveLink::CalcGrad(const Eigen::VectorXd &delta_p,const Eigen::VectorXd &theta) const{
	auto Pos_Deriv = CalcPos_Deriv1(theta);
	VectorXd grad(theta.size());
	
	grad = TargWeight * (delta_p.transpose() * Pos_Deriv).transpose();
	grad += Wreg * theta;
	return grad;
}

MatrixXd ObjectiveLink::CalcPos_Deriv1(const Eigen::VectorXd &theta) const {
	VectorXd current_posDeriv(2);
	MatrixXd P_prime(2,theta.size());
	MatrixXd T;
	MatrixXd R;
	MatrixXd mat_prod;
	for(int i = 0; i < theta.size(); i++){
		//caclulates the derivative for p_j
		mat_prod = MatrixXd::Identity(3,3);

		for (int j = 0; j < theta.size(); j++)
		{
			//Uses identity for transform 
			T = Eigen::MatrixXd::Identity(3,3);
			if(j > 0){
				//sets Tx and Ty if not first (T_1)
				T(0,2) = this->lengths.at(i);
				T(1,2) = 0;
			}
			//uses the derivative if it is the current link 
			R = Eigen::MatrixXd::Identity(3,3);
			if(j == i){
				R(0,0) = -sin(theta(j));
				R(0,1) = -cos(theta(j));
				R(1,0) = cos(theta(j));
				R(1,1) = -sin(theta(j));
				//removes homogenous coord on derivate
				R(2,2) = 0;
			}
			else{
				R(0,0) = cos(theta(j));
				R(0,1) = -sin(theta(j));
				R(1,0) = sin(theta(j));
				R(1,1) = cos(theta(j));
			}

			mat_prod = mat_prod * T * R;
		}
		
		current_posDeriv = (mat_prod * r).segment<2>(0);
		P_prime.block<2,1>(0,i) = current_posDeriv;
	}

	return P_prime;
}


MatrixXd ObjectiveLink::CalcPos_Deriv2(const Eigen::VectorXd &theta, const Eigen::VectorXd &delta_p) const{
	VectorXd current_posDeriv(2);
	MatrixXd P_2prime(2*theta.size(),theta.size());
	MatrixXd T;
	MatrixXd R;
	MatrixXd mat_prod;

	for(int i = 0; i < theta.size(); i++){
		//caclulates the derivative for p_j
		mat_prod = MatrixXd::Identity(3,3);

		for (int j = 0; j < theta.size(); j++)
		{
			//Uses identity for transform 
			T = Eigen::MatrixXd::Identity(3,3);
			if(j > 0){
				//sets Tx and Ty if not first (T_1)
				T(0,2) = this->lengths.at(j);
				T(1,2) = 0;
			}
			//uses the derivative if it is the current link 
			R = Eigen::MatrixXd::Identity(3,3);
			//calculates the 2nd derivate 
			if(j == i) {
				R(0,0) = -cos(theta(j));
				R(0,1) = sin(theta(j));
				R(1,0) = -sin(theta(j));
				R(1,1) = -cos(theta(j));
				//removes homogenous coord on derivate
				R(2,2) = 0;
			}
			else {
				R(0,0) = cos(theta(j));
				R(0,1) = -sin(theta(j));
				R(1,0) = sin(theta(j));
				R(1,1) = cos(theta(j));
			}

			mat_prod = mat_prod * T * R;
		}
		
		current_posDeriv = (mat_prod * r).segment<2>(0);
	}
	return P_2prime;
}