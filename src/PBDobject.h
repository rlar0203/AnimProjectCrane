#pragma once
#ifndef PBDOBJECT_H
#define PBDOBJECT_H 

#include <vector>
#include <memory>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include <Eigen/Sparse>

class Particle;
class Spring;
class MatrixStack;
class Program;

class pbdObject
{
private:
    int rows;
    int cols;
    std::vector< std::shared_ptr<Particle> > particles;
    std::vector< std::shared_ptr<Spring> > springs;

    std::vector<unsigned int> eleBuf;
    std::vector<float> posBuf;
    std::vector<float> norBuf;
    std::vector<float> texBuf;
    unsigned eleBufID;
    unsigned posBufID;
    unsigned norBufID;
    unsigned texBufID;
public:
    pbdObject( int rows, int cols,
        const Eigen::Vector3d &x00,
        const Eigen::Vector3d &x01,
        const Eigen::Vector3d &x10,
        const Eigen::Vector3d &x11,
        double mass,
        double alpha,
        double damping,
        double p_radius);
   virtual ~pbdObject();

   void tare();
   void reset();
   void updatePosNor();
   void step(double h, const Eigen::Vector3d &grav);

   void init();
   void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p) const;
};





#endif 