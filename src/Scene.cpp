#include <iostream>

#include "GLSL.h"
#include "Scene.h"
#include "Particle.h"
#include "PBDobject.h"
#include "Shape.h"
#include "Program.h"

using namespace std;
using namespace Eigen;

Scene::Scene() :
	t(0.0),
	h(1e-2),
	grav(0.0, 0.0, 0.0)
{
}

Scene::~Scene()
{
}

void Scene::load(const string &RESOURCE_DIR)
{
	// Units: meters, kilograms, seconds
	h = 1e-4;
	
	grav << 0.0, -9.8, 0.0;
	//EDIT this to change cloth row/columns
	int rows = 4;
	int cols = 4;

	double mass = 0.1;
	double alpha = 0;
	double damping = 1e-2;
	double pradius = 0.01; // Particle radius, used for collisions
	Vector3d x00(-0.5, 0.5, 0.6);
	Vector3d x01(0.5, 0.5, 0.6);
	Vector3d x10(-0.5, -0.5, 0.6);
	Vector3d x11(0.5, -0.5, 0.6);
	object = make_shared<pbdObject>(rows, cols, x00, x01, x10, x11, mass, alpha, damping, pradius);
	

}

void Scene::init()
{
	
	object->init();
}

void Scene::tare()
{

	object->tare();
}

void Scene::reset()
{
	t = 0.0;

	object->reset();
}

void Scene::step()
{
	t += h;
	
	// Simulate the cloth
	object->step(h, grav);
}

void Scene::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> prog) const
{
	object->draw(MV, prog);
}
