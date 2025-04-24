#include <iostream>
#include <fstream>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "PBDobject.h"
#include "Particle.h"
#include "Spring.h"
#include "MatrixStack.h"
#include "Program.h"
#include "GLSL.h"

using namespace std;
using namespace Eigen;



pbdObject::pbdObject( int rows, int cols,
    /*
    NOTE the x00 -> x11 will be handled in the exact sub shape in 
    the future to allow custom objects and only the particle array
    will be passed through 
    =================================================================
    */
    const Eigen::Vector3d &x00,
    const Eigen::Vector3d &x01,
    const Eigen::Vector3d &x10,
    const Eigen::Vector3d &x11,
    /*=========================================================*/
    double mass,
    double alpha,
    double damping,
    double p_radius) {

    assert(rows > 1);
	assert(cols > 1);
	assert(mass > 0.0);
	assert(alpha >= 0.0);
	assert(damping >= 0.0);
	assert(p_radius >= 0.0);

    this->rows = rows;
    this->cols = cols;
    
    /*
    ==================================

    this should be moved to the individual type classes
    with it's own 
     and should 
    only be here for initial creation and testing

    ================================
    */
   
 //****************this is array construction is for a quad ****************

    double x_step = (x01(0) - x00(0))/(cols-1);
    double y_step = (x10(1) - x00(1))/(rows-1);

    //create particles 
    int nVerts = rows*cols;
    for (int i = 0; i < rows; i++) {
        for(int j = 0; j < cols; ++j) {
            auto p = make_shared<Particle>();
            particles.push_back(p);
            p->r = p_radius;
            p->d = damping;

            Vector3d position;
            // since 2d z will be all the same
            position(0) = x_step * j + (x00(0));
            position(1) = y_step * i + (x00(1));
            position(2) = x00(2);

            p->x = position;
            p->fixed = false;

            //handles corner assignment
            // AND fixing which is only to be use for testing puposes
			if(i == 0) {
				if(j == 0) {
					p->x = x00;
					
				} else if(j == (cols-1)){
					p->x = x01;	
				}
				else if (j < cols/2 )
				{
					p->fixed  = true;
				}
				
			} 
			else if(i == (rows-1)) {
				if(j == 0) {
					p->x = x10;
				} else if(j == (cols-1)){
					p->x = x11;
				}
				
			}

            p->m = mass/ (double) (rows*cols);
            
			


        }
    }
    
    // create x springs (for quad )
    for(int p = 1; p < rows*cols; p++){
		//skips if we are trying to use a starting point in a row (@col 0) for the final point
		if( (p)%cols == 0){
			continue;
		}
		int init = (p) - 1;
		int final = (p);
		springs.push_back(make_shared<Spring>(particles[init],particles[final],alpha));
	}

    // create y springs bending (for quad)
	for(int p = 0; p < (rows*cols) - (cols*2); ++p) {
		int inital = p;
		int final = p + (cols*2);
		
		springs.push_back(make_shared<Spring>(particles[inital],particles[final],alpha));
	}

    // Create shear springs
	//stops at the end of the second to last row 
	for(int p = 0; p < ((rows*cols) - rows); ++p ) {
		int inital = p;
		int final;
		//if the point is col 0 of the row only does the one diagonal spring 
		if(p % cols == 0) {
			final = p + (cols + 1);
			springs.push_back(make_shared<Spring>(particles[inital],particles[final],alpha));
		
		}
		// else if point is the final col of the rows then it will do the one diagonal spring going back 
		else if(p % cols == (cols - 1) ) {
			final = p + (cols - 1);
			springs.push_back(make_shared<Spring>(particles[inital],particles[final],alpha));
		}
		//else just do both diagonal springs
		else {
			final = p + (cols - 1);
			springs.push_back(make_shared<Spring>(particles[inital],particles[final],alpha));
			final = p + (cols + 1);
			springs.push_back(make_shared<Spring>(particles[inital],particles[final],alpha));
		}
    }

	/*
		// Create x bending springs
		for(int p = 2; p < (rows*cols); ++p) {
			//skips if the final point of the bending spring is in the first two cols
			if((p%cols == 0) || (p%cols == 1)){
				continue;
			}
			int inital = p-2;
			int final = p;
			springs.push_back(make_shared<Spring>(particles[inital],particles[final],alpha));
	
		}
		// Create y bending springs
		for(int p = 0; p < (rows*cols) - (cols*2); ++p) {
			int inital = p;
			int final = p + (cols*2);
			
			springs.push_back(make_shared<Spring>(particles[inital],particles[final],alpha));
		}
	*/
    // Build vertex buffers
	posBuf.clear();
	norBuf.clear();
	texBuf.clear();
	eleBuf.clear();
	posBuf.resize(nVerts*3);
	norBuf.resize(nVerts*3);
	updatePosNor();
	
	// Texture coordinates (don't change)
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			texBuf.push_back(i/(rows-1.0));
			texBuf.push_back(j/(cols-1.0));
		}
	}
	
	// Elements (don't change)
	for(int i = 0; i < rows-1; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k0 = i*cols + j;
			int k1 = k0 + cols;
			// Triangle strip
			eleBuf.push_back(k0);
			eleBuf.push_back(k1);
		}
	}
}

pbdObject::~pbdObject(){ }

void pbdObject::tare() {
    for(auto p : particles) {
        p->tare();
    }
};

void pbdObject::reset() {
    for(auto p : particles) {
        p->reset();
    }
    updatePosNor();
}

void pbdObject::updatePosNor() {
	// Position
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k = i*cols + j;
			Vector3d x = particles[k]->x; // updated position
			posBuf[3*k+0] = x(0);
			posBuf[3*k+1] = x(1);
			posBuf[3*k+2] = x(2);
		}
	}
	
	// Normal
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			// Each particle has four neighbors
			//
			//      v1
			//     / | \
			// u0 /__|__\ u1
			//    \  |  /
			//     \ | /
			//      v0
			//
			// Use these four triangles to compute the normal
			int k = i*cols + j;
			int ku0 = k - 1;
			int ku1 = k + 1;
			int kv0 = k - cols;
			int kv1 = k + cols;
			Vector3d x = particles[k]->x;
			Vector3d xu0, xu1, xv0, xv1, dx0, dx1, c;
			Vector3d nor(0.0, 0.0, 0.0);
			int count = 0;
			// Top-right triangle
			if(j != cols-1 && i != rows-1) {
				xu1 = particles[ku1]->x;
				xv1 = particles[kv1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Top-left triangle
			if(j != 0 && i != rows-1) {
				xu1 = particles[kv1]->x;
				xv1 = particles[ku0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-left triangle
			if(j != 0 && i != 0) {
				xu1 = particles[ku0]->x;
				xv1 = particles[kv0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-right triangle
			if(j != cols-1 && i != 0) {
				xu1 = particles[kv0]->x;
				xv1 = particles[ku1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			nor /= count;
			nor.normalize();
			norBuf[3*k+0] = nor(0);
			norBuf[3*k+1] = nor(1);
			norBuf[3*k+2] = nor(2);
		}
	}

}

void pbdObject::step(double h, const Vector3d &grav) {
    //for 2D set all Z forces and velocities to 0
    //updates the position of each particle
    for(int i = 0; i < (int)this->particles.size(); ++i){
    Vector3d force;
    auto p = this->particles.at(i);
    if(p->fixed){
        p->p = p->x;
        continue;
    }
    force = p->m * grav - p->d * p->v;
    p->v += (h/p->m) * force;
    p->p = p->x;
    p->x += h*p->v;

    }
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++
    // +             Insert Collision check here          +
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++

    // NOTE THIS ONLY WORKS FOR SQUARE and doesn't take into account friction
    //loops through all the spring constraints and updates the positions accordingly
	for(int j = 0; j < (int)this->springs.size(); ++j) {
		auto spring = this->springs.at(j);

		Vector3d delt_x = spring->p1->x - spring->p0->x;
		double len = (delt_x).norm();
		double C = len - spring->L;
		Vector3d grad_C0 = -delt_x/len;
		Vector3d grad_C1 = delt_x/len;
		//can use 2 for denominator due to all mass being equal
		// as w_i = 1/m_i and grad_C is a unit vector so .norm() = 1
		double lam_w;
		if(spring->p0->fixed || spring->p1->fixed){
			lam_w = -C / ( 1 + spring->alpha/(h*h));
		}
		else{
			lam_w = -C / (2 + spring->alpha/(h*h));
		}
		//========================== CHATGPT: fixed check done by GPT ===============================
		if(!spring->p0->fixed){
			spring->p0->x += lam_w * grad_C0;
		}
		if(!spring->p1->fixed){
			spring->p1->x += lam_w * grad_C1;
		}
    }

    updatePosNor();

}


//WILL HAVETO CHANGE THIS IF WE WANT DEBUGGING SUCH AS WIRE FRAME VIEW OR NON 3D SHAPES
void pbdObject::init() {
    glGenBuffers(1, &posBufID);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &norBufID);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &texBufID);
	glBindBuffer(GL_ARRAY_BUFFER, texBufID);
	glBufferData(GL_ARRAY_BUFFER, texBuf.size()*sizeof(float), &texBuf[0], GL_STATIC_DRAW);
	
	glGenBuffers(1, &eleBufID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size()*sizeof(unsigned int), &eleBuf[0], GL_STATIC_DRAW);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	assert(glGetError() == GL_NO_ERROR);
}


void pbdObject::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> p) const
{
	// Draw mesh
	glUniform3f(p->getUniform("kdFront"), 0.894f, 0.882f, 0.792f);
	glUniform3f(p->getUniform("kdBack"),  0.776f, 0.843f, 0.835f);
	MV->pushMatrix();
	glUniformMatrix4fv(p->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	int h_pos = p->getAttribute("aPos");
	glEnableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	int h_nor = p->getAttribute("aNor");
	glEnableVertexAttribArray(h_nor);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	int h_tex = p->getAttribute("aTex");
	if(h_tex >= 0) {
		glEnableVertexAttribArray(h_tex);
		glBindBuffer(GL_ARRAY_BUFFER, texBufID);
		glVertexAttribPointer(h_tex, 2, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	}
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	for(int i = 0; i < rows; ++i) {
		glDrawElements(GL_TRIANGLE_STRIP, 2*cols, GL_UNSIGNED_INT, (const void *)(2*cols*i*sizeof(unsigned int)));
	}
	if(h_tex >= 0) {
		glDisableVertexAttribArray(h_tex);
	}
	glDisableVertexAttribArray(h_nor);
	glDisableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	MV->popMatrix();
}