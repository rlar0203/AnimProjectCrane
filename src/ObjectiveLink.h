#pragma once
#ifndef OBJECTIVE_LINK_H
#define OBJECTIVE_LINK_H

#include "Objective.h"

class ObjectiveLink : public Objective
{
public:
    ObjectiveLink(Eigen::Vector3d weights, int num_links,const Eigen::VectorXd pos_targ, std::vector<double> _lengths);    
    virtual ~ObjectiveLink();
	virtual double evalObjective(const Eigen::VectorXd &x) const;
	virtual double evalObjective(const Eigen::VectorXd &x, Eigen::VectorXd &g) const;
	virtual double evalObjective(const Eigen::VectorXd &x, Eigen::VectorXd &g, Eigen::MatrixXd &H) const;
    Eigen::VectorXd CalcDeltPos(const Eigen::VectorXd &pos_targ, const Eigen::VectorXd &theta) const;
    Eigen::VectorXd CalcGrad(const Eigen::VectorXd &delta_p,const Eigen::VectorXd &theta) const;
    Eigen::MatrixXd CalcHess(const Eigen::VectorXd &delta_p) const;
    Eigen::MatrixXd CalcPos_Deriv1(const Eigen::VectorXd &theta) const;
    Eigen::MatrixXd CalcPos_Deriv2(const Eigen::VectorXd &theta, const Eigen::VectorXd &delta_p) const;
private:
    double TargWeight;
    int num_links; //number of links
    std::vector<double> lengths;
    Eigen::VectorXd pos_targ;
    Eigen::VectorXd r; //effort vector with homogenous coord
    Eigen::MatrixXd Wreg;
};

#endif 