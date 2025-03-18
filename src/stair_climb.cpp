#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include "leg_model.hpp"
#include "stair_climb.hpp"

StairClimb::StairClimb(bool sim, double CoM_bias, int rate, double BL, double BW, double BH) : 
    /* Initializer List */
    leg_model(sim),
    CoM_bias(CoM_bias),
    rate(rate), 
    BL(BL),
    BW(BW),
    BH(BH)
{
    // Initialize positions
}//end StairClimb

void StairClimb::initialize(double init_eta[8]) {
    double init_theta[4] = {init_eta[0], init_eta[2], init_eta[4], init_eta[6]};
    double init_beta[4]  = {-init_eta[1], init_eta[3], init_eta[5], -init_eta[7]};
    // Get foothold in hip coordinate from initial configuration
    double relative_foothold[4][2] = {};
    int current_rim = 0;
    for (int i=0; i<4; i++) {
        leg_model.contact_map(init_theta[i], init_beta[i]);
        current_rim = leg_model.rim;
        leg_model.forward(init_theta[i], init_beta[i]);
        if (current_rim == 1) {
            relative_foothold[i][0] = leg_model.U_l[0];
        } else if (current_rim == 2) {
            relative_foothold[i][0] = leg_model.L_l[0];
        } else if (current_rim == 3) {
            relative_foothold[i][0] = leg_model.G[0];
        } else if (current_rim == 4) {
            relative_foothold[i][0] = leg_model.L_r[0];
        } else if (current_rim == 5) {
            relative_foothold[i][0] = leg_model.U_r[0];
        } else {
            std::cout << "Leg cannot contact ground if use the given initial theta/beta." << std::endl;
        }//end if else
        relative_foothold[i][1] = -stand_height;
    }//end for
    // Get first swing leg  
    int first_swing_leg;
    if (relative_foothold[0][0] < relative_foothold[1][0]) {
        first_swing_leg = 0;
    } else {
        first_swing_leg = 1;
    }//end if else
    // Get foothold in world coordinate
    CoM = {CoM_bias, stand_height};
    hip = {{{BL/2, stand_height} ,
            {BL/2, stand_height} ,
            {-BL/2, stand_height},
            {-BL/2, stand_height}}};
    next_hip = hip;
    // Initial leg configuration
    for (int i=0; i<4; i++) {
        foothold[i] = {next_hip[i][0] + relative_foothold[i][0] + CoM_bias, next_hip[i][1] + relative_foothold[i][1]};
    }//end for
    // Initial theta/beta
    for (int i=0; i<4; i++) {
        theta[i] = init_theta[i];
        beta[i]  = init_beta[i];
    }//end for
}//end initialize

std::array<std::array<double, 4>, 2> StairClimb::step() {


}//end step


void StairClimb::move_CoM_stable(std::vector<LegInfo>& leg_info, int swing_leg, Eigen::Vector2d& CoM, double pitch) {
    Eigen::Matrix2d rotation;
    rotation << std::cos(pitch), -std::sin(pitch),
                std::sin(pitch),  std::cos(pitch);
    Eigen::Vector2d CoM_offset = rotation * CoM_bias;

    int move_dir = (leg_info[swing_leg].ID == 0 || leg_info[swing_leg].ID == 1) ? -1 : 1;
    
    while (move_dir * (CoM[0] + CoM_offset[0]) < move_dir * ((leg_info[(swing_leg + 1) % 4].foothold[0] + leg_info[(swing_leg - 1) % 4].foothold[0]) / 2) + stability_margin) {
        if (move_dir * velocity[0] < max_velocity) {
            velocity[0] += move_dir * acc / sampling;
        }
        CoM += velocity / sampling;
        
        for (int i = 0; i < 4; ++i) {
            Eigen::Vector2d hip = leg_info[i].hip_position(CoM, pitch);
            std::cout << "Hip position for leg " << i << ": " << hip.transpose() << std::endl;
        }
    }
}//end move_CoM_stable

void StairClimb::move_CoM_stable_smooth_fixed_leg_length(std::vector<LegInfo>& leg_info, int swing_leg, Eigen::Vector2d& CoM, double pitch) {
    Eigen::Matrix2d rotation;
    rotation << std::cos(pitch), -std::sin(pitch),
                std::sin(pitch),  std::cos(pitch);
    Eigen::Vector2d CoM_offset = rotation * CoM_bias;

    int move_dir = (leg_info[swing_leg].ID == 0 || leg_info[swing_leg].ID == 1) ? -1 : 1;
    LegModel leg_model;
    leg_model.forward(0.0, 0.0); // 初始化腿部模型
    double leg_length = leg_model.G.norm();
    
    while (move_dir * (CoM[0] + CoM_offset[0]) < move_dir * ((leg_info[(swing_leg + 1) % 4].foothold[0] + leg_info[(swing_leg - 1) % 4].foothold[0]) / 2) + stability_margin) {
        if (move_dir * velocity[0] < max_velocity) {
            velocity[0] += move_dir * acc / sampling;
        }
        CoM += velocity / sampling;
        
        double hip_x = CoM[0];
        double hip_y;
        leg_model.forward(0.0, 0.0);
        if (leg_info[swing_leg].contact_edge) {
            hip_y = CoM[1];
        } else {
            hip_y = CoM[1] + leg_model.G[1] + std::sqrt(leg_length * leg_length - std::pow(hip_x - (CoM[0] + leg_model.G[0]), 2));
        }
        
        Eigen::Vector2d hip = Eigen::Vector2d(hip_x, hip_y);
        std::cout << "Hip position for swing leg " << swing_leg << ": " << hip.transpose() << std::endl;
    }
    CoM = Eigen::Vector2d(CoM[0], CoM[1]);
    pitch = std::asin((CoM[1] - CoM[1]) / BL);
}//end move_CoM_stable_fixed_leg_length

void StairClimb::swing_same_step(std::vector<LegInfo>& leg_info, int swing_leg, Eigen::Vector2d& CoM, double pitch, double front_height, double hind_height) {
    double final_CoM_height = (front_height + hind_height) / 2;
    double margin_d = std::abs(CoM[0] - (leg_info[(swing_leg+1)%4].foothold[0] + leg_info[(swing_leg-1)%4].foothold[0]) / 2) - min_margin;
    
    Eigen::Vector2d p_lo = leg_info[swing_leg].hip_position(CoM, pitch); 
    Eigen::Vector2d p_td = leg_info[swing_leg].next_foothold;
    SwingProfile sp(p_td[0] - p_lo[0], 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, p_lo[0], p_lo[1], p_td[1] - p_lo[1]);
    
    double t_f_x = margin_d / max_velocity;
    double coeff_b = acc;
    double coeff_a = -std::sqrt(std::abs(std::pow(coeff_b, 3) / (6 * (final_CoM_height - CoM[1]))));
    double t_f_y = -coeff_b / coeff_a;
    double t_f = std::max({min_swing_time_step, t_f_x, t_f_y});
    double local_max_velocity = margin_d / t_f;
    int samples = static_cast<int>(t_f * sampling);
    
    for (int sample = 0; sample < samples; ++sample) {
        if (velocity[0] < local_max_velocity) {
            velocity[0] += acc / sampling;
        } else if (velocity[0] > local_max_velocity + acc / sampling) {
            velocity[0] -= acc / sampling;
        }
        
        double t_ = static_cast<double>(sample+1) / sampling;
        velocity[1] = (t_ <= t_f_y) ? coeff_a * (t_ * t_) + coeff_b * t_ : 0.0;
        CoM += velocity / sampling;
        
        if (swing_leg == 0 || swing_leg == 1) {
            pitch = std::asin((CoM[1] - hind_height) / (BL / 2));
        } else {
            pitch = std::asin((front_height - CoM[1]) / (BL / 2));
        }
        
        Eigen::Vector2d hip = leg_info[swing_leg].hip_position(CoM, pitch);
        double swing_phase_ratio = static_cast<double>(sample+1) / samples;
        Eigen::Vector2d curve_point = sp.getFootendPoint(swing_phase_ratio);
        
        std::cout << "Swing leg " << swing_leg << " moving to: " << curve_point.transpose() << std::endl;
    }
    velocity[1] = 0.0;
}//end swing_same_step

void StairClimb::swing_next_step(std::vector<LegInfo>& leg_info, int swing_leg, Eigen::Vector2d& CoM, double pitch, double front_height, double hind_height) {
    double lift_height = 0.04;
    bool is_clockwise = (swing_leg == 0 || swing_leg == 1) ? (leg_info[0].ID == leg_info[1].ID) : (leg_info[2].ID == leg_info[3].ID);
    double final_CoM_height = (front_height + hind_height) / 2;
    
    double t_f = is_clockwise ? min_swing_time_cw : min_swing_time_ccw;
    int samples = static_cast<int>(t_f * sampling);
    
    double t_f_x = std::abs(velocity[0] / acc);
    double coeff_b = acc;
    double coeff_a = -std::sqrt(std::abs(std::pow(coeff_b, 3) / (6 * (final_CoM_height - CoM[1]))));
    double t_f_y = -coeff_b / coeff_a;
    
    LegModel leg_model;
    leg_model.forward(0.0, 0.0);
    Eigen::Vector2d final_CoM = Eigen::Vector2d(CoM[0] + velocity[0] * t_f_x - std::copysign(acc * (t_f_x * t_f_x) / 2, velocity[0]), final_CoM_height);
    double final_pitch = std::asin((front_height - hind_height) / BL);
    Eigen::Vector2d final_hip = leg_info[swing_leg].hip_position(final_CoM, final_pitch);
    
    for (int sample = 0; sample < samples; ++sample) {
        if (velocity[0] > acc / sampling) {
            velocity[0] -= acc / sampling;
        } else if (velocity[0] < -acc / sampling) {
            velocity[0] += acc / sampling;
        } else {
            velocity[0] = 0.0;
        }
        
        double t_ = static_cast<double>(sample + 1) / sampling;
        velocity[1] = (t_ <= t_f_y) ? coeff_a * (t_ * t_) + coeff_b * t_ : 0.0;
        CoM += velocity / sampling;
        
        if (swing_leg == 0 || swing_leg == 1) {
            pitch = std::asin((CoM[1] - hind_height) / (BL / 2));
        } else {
            pitch = std::asin((front_height - CoM[1]) / (BL / 2));
        }
        
        Eigen::Vector2d hip = leg_info[swing_leg].hip_position(CoM, pitch);
        leg_model.inverse(final_hip - hip, "G");
        std::cout << "Swing leg " << swing_leg << " moving to: " << hip.transpose() << std::endl;
    }
    velocity[1] = 0.0;
}//end swing_next_step
