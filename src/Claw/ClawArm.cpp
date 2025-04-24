
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


#include "ClawArm.h"
#include "../Particle.h"
#include "../Spring.h"
#include "../MatrixStack.h"
#include "../Program.h"
#include "../GLSL.h"

using namespace std;
using namespace glm;
using namespace Eigen;

ClawArm::draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p) const {
    MV->pushMatrix();
    glColor3d(0.0,0.0,1.0);
    glBegin(GL_LINE_STRIP);
    glVertex3d(origin.x,origin.y,origin.z);
    glVertex3d(Midjoint.x,Midjoint.y,Midjoint.z);
    glVertex3d(EndPoint.x,EndPoint.y,EndPoint.z);
    glEnd();
    MV->popMatrix();
}

ClawArm::ClawArm(Vector3d startpoint) {
}

ClawArm::~ClawArm() {
}