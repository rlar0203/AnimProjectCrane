#pragma once
#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <memory>
#include <Eigen/Dense>

class Objective;

class Optimizer
{
public:
	Optimizer();
	~Optimizer();
	virtual Eigen::VectorXd optimize(const std::shared_ptr<Objective> objective, const Eigen::VectorXd &xInit) = 0;
	
	void setTol(double tol) { this->tol = tol; }
	void setIterMax(int iterMax) { this->iterMax = iterMax; }
	int getIter() const { return iter; }
	void setAlphaInit(double alphaInit) { this->alphaInit = alphaInit; }
	void setGamma(double gamma) { this->gamma = gamma; }
	void setIterMaxLS(int iterMaxLS) { this->iterMaxLS = iterMaxLS; }
	int getIterLS() const { return iterLS; }
	
protected:
	double tol;
	int iterMax;
	int iter;
	double alphaInit;
	double gamma;
	int iterMaxLS;
	int iterLS;
};

#endif
