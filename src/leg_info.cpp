#include <cmath>
#include <array>

#include "leg_info.hpp"

LegInfo::LegInfo(int leg_ID, std::array<double, 2> init_foothold, std::array<double, 2> CoM, double BL) : 
    /* Initializer List */
    ID(leg_ID), 
    BL(BL), 
    touch(true), 
    next_up(false), 
    one_step(true), 
    contact_edge(false), 
    contact_alpha(0.0), 
    stair(0) 
{
    std::array<double, 2> hip_p = get_hip_position(CoM, 0.0);
    foothold = {hip_p[0] + init_foothold[0], hip_p[1] + init_foothold[1]};
    next_foothold = foothold;
}//end LegInfo

std::array<double, 2> LegInfo::get_hip_position(std::array<double, 2> CoM, double pitch) {
    std::array<double, 2> offset = {BL/2.0 * std::cos(pitch), BL/2.0 * std::sin(pitch)};
    if (ID==0 || ID==1) {
        return {CoM[0] + offset[0], CoM[1] + offset[1]};
    } else {    // ID==2 || ID==3
        return {CoM[0] - offset[0], CoM[1] - offset[1]};
    }//end if else
}//end get_hip_position