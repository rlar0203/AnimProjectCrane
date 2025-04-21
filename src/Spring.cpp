#include "Spring.h"
#include "Particle.h"

using namespace std;
using namespace Eigen;

Spring::Spring(shared_ptr<Particle> p0, shared_ptr<Particle> p1, double alpha)
{
	assert(p0);
	assert(p1);
	assert(p0 != p1);
	this->p0 = p0;
	this->p1 = p1;
	this->alpha = alpha;
	
	// Computes L
	this->L = (p1->x - p0->x).norm();
	
}

Spring::~Spring()
{
	
}
