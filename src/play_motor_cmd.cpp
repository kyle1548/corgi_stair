#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include "corgi_msgs/MotorCmdStamped.h"
#include "corgi_msgs/TriggerStamped.h"


corgi_msgs::TriggerStamped trigger_msg;
void trigger_cb(const corgi_msgs::TriggerStamped msg) {
    trigger_msg = msg;
}//end trigger_cb

bool read_line(std::ifstream& file, corgi_msgs::MotorCmdStamped& msg) {
    std::string line;
    if (!std::getline(file, line)) return false;

    std::istringstream ss(line);
    std::string token;
    std::getline(ss, token, ',');  // time
    double time = std::stod(token);
    msg.header.stamp = ros::Time(time);

    auto& A = msg.module_a;
    auto& B = msg.module_b;
    auto& C = msg.module_c;
    auto& D = msg.module_d;
    std::vector<double*> fields = {
        &A.theta, &A.beta, &A.kp_r, &A.kp_l, &A.ki_r, &A.ki_l, &A.kd_r, &A.kd_l, &A.torque_r, &A.torque_l,
        &B.theta, &B.beta, &B.kp_r, &B.kp_l, &B.ki_r, &B.ki_l, &B.kd_r, &B.kd_l, &B.torque_r, &B.torque_l,
        &C.theta, &C.beta, &C.kp_r, &C.kp_l, &C.ki_r, &C.ki_l, &C.kd_r, &C.kd_l, &C.torque_r, &C.torque_l,
        &D.theta, &D.beta, &D.kp_r, &D.kp_l, &D.ki_r, &D.ki_l, &D.kd_r, &D.kd_l, &D.torque_r, &D.torque_l,
    };

    for (auto* ptr : fields) {
        if (!std::getline(ss, token, ',')) return false;
        *ptr = std::stod(token);
    }

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_cmd_player");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<corgi_msgs::MotorCmdStamped>("motor/command", 1);
    ros::Subscriber trigger_sub = nh.subscribe<corgi_msgs::TriggerStamped>("trigger", 1, trigger_cb);

    std::ifstream file("motor_commands.csv");
    std::string header;
    std::getline(file, header); // skip header

    ros::Rate rate(1000);  // 控制重播速度

    while (ros::ok()) {
        ros::spinOnce();
        if (trigger_msg.enable) {
            corgi_msgs::MotorCmdStamped msg;
            if (!read_line(file, msg)) break;
            pub.publish(msg);
        }
        rate.sleep();
    }

    return 0;
}
