#include "Optimizer.h"

Optimizer::Optimizer() :
	tol(1e-6),
	iterMax(50),
	iter(0),
	alphaInit(1.0),
	gamma(0.5),
	iterMaxLS(20),
	iterLS(0)
{
	
}

Optimizer::~Optimizer()
{
	
}
