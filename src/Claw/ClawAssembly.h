#pragma once
#ifndef CLAW_ASSEMBLY_H
#define CLAW_ASSEMBLY_H 

#include <iostream>
#include <vector>


class ClawArm;

class ClawAssembly
{
private:
    std::vector<ClawArm> ClawArms;

public:
    ClawAssembly(/* args */);
    ~ClawAssembly();
};



#endif 