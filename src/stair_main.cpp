#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <array>
#include <string>
#include <chrono>
#include "ros/ros.h"

#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/SimDataStamped.h"
#include "corgi_msgs/TriggerStamped.h"
#include "leg_model.hpp"
#include "bezier.hpp"
#include "walk_gait.hpp"
#include "stair_climb.hpp"

#define INIT_THETA (M_PI*17.0/180.0)
#define INIT_BETA (0.0)

corgi_msgs::TriggerStamped trigger_msg;
corgi_msgs::SimDataStamped sim_data;

void trigger_cb(const corgi_msgs::TriggerStamped msg) {
    trigger_msg = msg;
}//end trigger_cb

void robot_cb(const corgi_msgs::SimDataStamped msg) {
    sim_data = msg;
}//end robot_cb

int main(int argc, char** argv) {
    ros::init(argc, argv, "stair_climb");
    ros::NodeHandle nh;
    ros::Publisher motor_pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1);
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1, trigger_cb);
    ros::Subscriber robot_sub = nh.subscribe<corgi_msgs::SimDataStamped>("sim/data", 1, robot_cb);
    corgi_msgs::MotorCmdStamped motor_cmd;
    std::array<corgi_msgs::MotorCmd*, 4> motor_cmd_modules = {
        &motor_cmd.module_a,
        &motor_cmd.module_b,
        &motor_cmd.module_c,
        &motor_cmd.module_d
    };
    // pid command
    for (int i=0; i<4; i++) {
        motor_cmd_modules[i]->kp_r = 90;
        motor_cmd_modules[i]->ki_r = 0;
        motor_cmd_modules[i]->kd_r = 1.0;
        motor_cmd_modules[i]->kp_l = 90;
        motor_cmd_modules[i]->ki_l = 0;
        motor_cmd_modules[i]->kd_l = 1.0;
        motor_cmd_modules[i]->torque_r = 0;
        motor_cmd_modules[i]->torque_l = 0;
    }//end for 

    /* Setting variable */
    double D=0.27, H=0.125;
    int stair_num = 3;
    enum STATES {INIT, TRANSFORM, WAIT, WALK, STAIR, END};
    const std::array<double, 2> CoM_bias = {0.0, 0.0};
    const int sampling_rate = 1000;
    const int transform_count = 2*sampling_rate; // 2s
    // double init_eta[8] = {1.7908786895256839, 0.7368824288764617, 1.1794001564068406, -0.07401410141135822, 1.1744876957173913, -1.8344700758454735e-15, 1.7909927830130310, 5.5466991499313485};
    // double init_eta[8] = {1.7695243267183387, 0.7277016876093340, 1.2151854401036246,  0.21018258666216960, 1.2151854401036246, -0.21018258666216960000, 1.7695243267183387, -0.727701687609334};   // normal
    double init_eta[8] = {1.8900999073259275, 0.5043376058303682, 1.6069784307289758, 0.13712110729189467, 1.6069784307289758, -0.13712110729189467, 1.8900999073259275, -0.5043376058303682};  // stand height 0.25, step length 0.3

    /* Initial variable */
    ros::Rate rate(sampling_rate);
    WalkGait walk_gait(true, CoM_bias[0], sampling_rate);
    StairClimb stair_climb(true, CoM_bias, sampling_rate);
    std::array<std::array<double, 4>, 2> eta_list = {{{INIT_THETA, INIT_THETA, INIT_THETA, INIT_THETA},
                                                      {INIT_BETA , INIT_BETA , INIT_BETA , INIT_BETA }}};   // init eta (wheel mode)
    
    /* Other variable */
    STATES state = INIT, last_state = INIT;
    double transform_ratio;
    bool trigger;
    int count;
    double pitch;
    double max_cal_time = 0.0;
    std::array<int, 4> swing_phase;

    /* Behavior loop */
    auto start = std::chrono::high_resolution_clock::now();
    walk_gait.set_velocity(0.1);
    walk_gait.set_stand_height(0.25);
    walk_gait.set_step_length(0.3);
    walk_gait.set_step_height(0.04);
    while (ros::ok()) {
        auto one_loop_start = std::chrono::high_resolution_clock::now();
        ros::spinOnce();
        if (state == END) {
            break;
        }//end if
        // state machine
        switch (state) {
            case INIT:
                transform_ratio = 0.0;
                trigger = false;
                count = 0;
                break;
            case TRANSFORM:
                transform_ratio += 1.0 / transform_count;
                for (int i=0; i<4; i++) {
                    eta_list[0][i] = INIT_THETA + transform_ratio * (init_eta[i*2]   - INIT_THETA);
                    eta_list[1][i] = INIT_BETA  + transform_ratio * (init_eta[i*2+1] - INIT_BETA);
                    eta_list[1][i] = (i == 1 || i == 2)? eta_list[1][i] : -eta_list[1][i];
                }//end for
                break;
            case WAIT:
                if (last_state != state) {
                    double current_eta[8] = {eta_list[0][0], -eta_list[1][0], eta_list[0][1], eta_list[1][1], eta_list[0][2], eta_list[1][2], eta_list[0][3], -eta_list[1][3]};
                    walk_gait.initialize(current_eta);
                }//end if
                break;
            case WALK:
                eta_list = walk_gait.step();
                break;
            case STAIR:
                count ++;
                if (last_state != state) {
                    double current_eta[8] = {eta_list[0][0], -eta_list[1][0], eta_list[0][1], eta_list[1][1], eta_list[0][2], eta_list[1][2], eta_list[0][3], -eta_list[1][3]};
                    stair_climb.initialize(current_eta);
                    for (int i=0; i<stair_num; i++) {
                        stair_climb.add_stair_edge(-D/2.0 + i*D - sim_data.position.x, (i+1)*H);
                    }//end for
                }//end if
                eta_list = stair_climb.step();
                pitch = stair_climb.get_pitch();
                break;
            default:
                break;
        }//end switch
        last_state = state;

        // next state
        switch (state) {
            case INIT:
                state = TRANSFORM;
                break;
            case TRANSFORM:
                if (transform_ratio > 1.0) {
                    state = WAIT;
                }//end if
                break;
            case WAIT:
                if (trigger_msg.enable) {
                    state = WALK;
                }//end if
                break;
            case WALK:
                /* Position feedback in Webots */
                // double hip_x = sim_data.position.x + 0.222;
                // if ( -D/2.0 - 0.35 - hip_x <= 0.3 ) {   // first stair edge - keep_d - front hip pos < max step length
                //     walk_gait.set_step_length((-D/2.0 - 0.35 - hip_x)/(0.2+0.4)); // step_length*(swing_phase + (1-swing_phase)/2) = foothold_x - hip_x
                //     std::array<int, 4> swing_phase = walk_gait.get_swing_phase();
                //     if (walk_gait.if_touchdown() && (swing_phase[0]==1 || swing_phase[1]==1)) { // hind leg touched down (front leg start to swing)
                //         state = STAIR;
                //     }//end if
                // }//end if
                swing_phase = walk_gait.get_swing_phase();
                if (walk_gait.if_touchdown() && (swing_phase[0]==1 || swing_phase[1]==1)) { // hind leg touched down (front leg start to swing)
                    double hip_x = sim_data.position.x + 0.222;
                    if (hip_x + 0.15 >= -D/2.0 - 0.15) {  
                        state = STAIR;
                    }//end if
                }//end if

                break;
            case STAIR:
                if (!stair_climb.if_any_stair()) {
                    state = END;
                }//end if  
                break;
            default:
                break;
        }//end switch

        /* Publish motor commands */
        for (int i=0; i<4; i++) {
            if (eta_list[0][i] > M_PI*160.0/180.0) {
                std::cout << "Exceed upper bound." << std::endl;
                eta_list[0][i] = M_PI*160.0/180.0;
            }//end if 
            if (eta_list[0][i] < M_PI*16.99/180.0) {
                std::cout << "Exceed lower bound." << std::endl;
                eta_list[0][i] = M_PI*16.99/180.0;
            }//end if 
            motor_cmd_modules[i]->theta = eta_list[0][i];
            motor_cmd_modules[i]->beta = (i == 1 || i == 2)? (eta_list[1][i]-pitch) : -(eta_list[1][i]-pitch);
        }//end for
        motor_pub.publish(motor_cmd);
        auto one_loop_end = std::chrono::high_resolution_clock::now();
        auto one_loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(one_loop_end - one_loop_start);
        if (one_loop_duration.count() > max_cal_time) {
            max_cal_time = one_loop_duration.count();
            std::cout << "max time: " << max_cal_time << " us" << std::endl;
        }//end if
        rate.sleep();
    }//end while
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "max time: " << max_cal_time << " us" << std::endl;
    std::cout << "time: " << duration.count() << " ms" << std::endl;
    
    ros::shutdown();
    return 0;
}//end main

