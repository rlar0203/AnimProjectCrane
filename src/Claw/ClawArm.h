#pragma once
#ifndef CLAW_ARM_H
#define CLAW_ARM_H


#include "../Particle.h"
#include "../Spring.h"

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include <Eigen/Sparse>

class Particle;
class Spring;
class MatrixStack;
class Program;
class GLSL;

class ClawArm
{
private:
    double armlength;
    Eigen::Vector3d origin(0.0,0.0,0.4);
    Eigen::Vector3d Midjoint(0.5,-0.5,0.4);
    Eigen::Vector3d EndPoint(0.5,-1.0,0.4);


public:
    ClawArm(Eigen::Vector3d startpoint);
    ~ClawArm();
    void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p) const;
};


#endif