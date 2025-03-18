#ifndef STAIRCLIMB_HPP
#define STAIRCLIMB_HPP

#include <array>
#include <complex>

#include "leg_model.hpp"
#include "leg_info.hpp"

class StairClimb {
    public:
        // Constructor
        StairClimb(bool sim=true, double CoM_bias=0.0, int rate=1000, double BL=0.444, double BW=0.4, double BH=0.2);

        std::array<std::array<double, 4>, 2> step();

    private:
        void move_CoM_stable(std::vector<LegInfo>& leg_info, int swing_leg, Eigen::Vector2d& CoM, double pitch);
        void move_CoM_stable_fixed_leg_length(std::vector<LegInfo>& leg_info, int swing_leg, Eigen::Vector2d& CoM, double pitch);
        void swing_next_step_smooth(std::vector<LegInfo>& leg_info, int swing_leg, Eigen::Vector2d& CoM, double pitch, double front_height, double hind_height);

        LegModel leg_model;
        LegInfo leg_info[4];
        std::array<double, 2> CoM;
        double pitch;
        double max_velocity;
        double stability_margin;
        Eigen::Vector2d CoM_bias;
        double acc;
        int sampling;
        Eigen::Vector2d velocity;
        double min_margin;
        double BL, BH, BW;
        double min_swing_time_cw, min_swing_time_ccw, min_swing_time_step;
};//end class StairClimb

#endif // STAIRCLIMB_HPP
