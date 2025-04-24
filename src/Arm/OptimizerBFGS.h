#pragma once
#ifndef OPTIMIZER_BFGS_H
#define OPTIMIZER_BFGS_H

#include "Optimizer.h"

class Objective;

class OptimizerBFGS : public Optimizer
{
public:
	OptimizerBFGS();
	virtual ~OptimizerBFGS();
	virtual Eigen::VectorXd optimize(const std::shared_ptr<Objective> objective, const Eigen::VectorXd &xInit);
};

#endif
