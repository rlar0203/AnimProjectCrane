#include <iostream>
#include <fstream>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>


#include "ClawAssembly.h"
#include "../Particle.h"
#include "../Spring.h"
#include "../MatrixStack.h"
#include "../Program.h"
#include "../GLSL.h"




using namespace std;
using namespace Eigen;

ClawAssembly::ClawAssembly(/* args */) {
    Vector3d origin(0,0,0);
}

ClawAssembly::~ClawAssembly()
{
}

