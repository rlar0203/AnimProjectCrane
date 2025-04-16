#pragma once
#ifndef OBJECTIVE_H
#define OBJECTIVE_H

#include <Eigen/Dense>

class Objective
{
public:
	Objective() {};
	virtual ~Objective() {};
	virtual double evalObjective(const Eigen::VectorXd &x) const = 0;
	virtual double evalObjective(const Eigen::VectorXd &x, Eigen::VectorXd &g) const = 0;
	virtual double evalObjective(const Eigen::VectorXd &x, Eigen::VectorXd &g, Eigen::MatrixXd &H) const = 0;
};

#endif
