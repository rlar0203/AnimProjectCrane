#pragma once
#ifndef OPTIMIZER_GD_H
#define OPTIMIZER_GD_H

#include "Optimizer.h"

class Objective;

class OptimizerGD : public Optimizer
{
public:
	OptimizerGD();
	virtual ~OptimizerGD();
	virtual Eigen::VectorXd optimize(const std::shared_ptr<Objective> objective, const Eigen::VectorXd &xInit);
};

#endif
