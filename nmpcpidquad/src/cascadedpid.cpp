#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <chrono>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <fstream>


template <typename T>
T clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max) ? max : value;
}

class PID {
public:
    PID(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), previous_error_(0), integral_(0) {}

    double calculate(double setpoint, double pv, double dt) {
        double error = setpoint - pv;
        double Pout = kp_ * error;
        integral_ += error * dt;
        integral_ = clamp(integral_, -0.075, 0.075);
        double Iout = ki_ * integral_;
        double derivative = (error - previous_error_) / dt;
        double Dout = kd_ * derivative;
        double output = Pout + Iout + Dout;
        previous_error_ = error;
        return output;
    }

private:
    double kp_, ki_, kd_;
    double previous_error_, integral_;
};

class PIDXY {
public:
    PIDXY(double kpphi, double kiphi,double kdphi, double kptheta, double kitheta, double kdtheta,  double kpx, double kdx, double kpy, double kdy) 
        : kpphi_(kpphi), kiphi_(kiphi), kdphi_(kdphi),  kptheta_(kptheta), kitheta_(kitheta), kdtheta_(kdtheta),  kpx_(kpx), kdx_(kdx), kpy_(kpy), kdy_(kdy), 
          previous_error_x(0), previous_error_phid(0), previous_error_y(0), previous_error_thetad(0), integralphid(0), integralthetad(0) {}

    std::tuple<double, double, double, double> calculatexy(double setpointx, double setpointy, double pvx, double pvy, double pvphid, double pvthetad, double pvphi, double pvtheta, double psi, double dt) {
        // Calculate error
        double errorx = setpointx - pvx;
        double errory = setpointy - pvy;
        double derivativex = (errorx - previous_error_x) / dt;
        double derivativey = (errory - previous_error_y) / dt;
        previous_error_x = errorx;
        previous_error_y = errory;

        // Proportional term
        double thetade = kpx_ * errorx + kdx_ * derivativex;
        double phide = -kpy_ * errory - kdy_ * derivativey;
        double phir = phide * cos(psi) + thetade * sin(psi);
        double thetar = -phide * sin(psi) + thetade * cos(psi);

        thetar = clamp(thetar, -0.1, 0.1);
        phir = clamp(phir, -0.1, 0.1);

        double phird = 1.25 * (phir - pvphi);
        double thetard = 1.25 * (thetar - pvtheta);

        phird = clamp(phird, -0.3, 0.3);
        thetard = clamp(thetard, -0.3, 0.3);

        double errorphid = phird - pvphid;
        double errorthetad = thetard - pvthetad;
        double derivativephid = (errorphid - previous_error_phid) / dt;
        double derivativethetad = (errorthetad - previous_error_thetad) / dt;
        integralphid += errorphid * dt;
        integralphid = clamp(integralphid, -0.05, 0.05);
        integralthetad += errorthetad * dt;
        integralthetad = clamp(integralthetad, -0.05, 0.05);
        previous_error_phid = errorphid;
        previous_error_thetad = errorthetad;

        double U2 = kpphi_ * errorphid + kdphi_ * derivativephid + kiphi_*integralphid;
        double U3 = kptheta_ * errorthetad + kdtheta_ * derivativethetad + kitheta_ * integralthetad;
        U2 = clamp(U2, -0.1, 0.1);
        U3 = clamp(U3, -0.1, 0.1);

        return {U2, U3, phir, thetar};
    }

private:
    double kpx_, kdx_, kpy_, kdy_;
    double kdphi_, kpphi_, kdtheta_, kptheta_, kiphi_, kitheta_;
    double previous_error_x, previous_error_phid;
    double integralphid, integralthetad;
    double previous_error_y, previous_error_thetad;
};

class PIDcontroller : public rclcpp::Node {
public:
    PIDcontroller(): Node("PIDcontroller"), controlz_(8.0, 0, 5.75), controlpsi_(3.25, 0.5, 1.75), controlxy_(0.75, 0.05, 0.05, 0.75, 0.05, 0.05, 0.05, 0.25, 0.05, 0.25), xdes(5.0), ydes(5.0), zdes(5.0),
    x(0), y(0), z(0), xd(0), yd(0), zd(0), phi(0), theta(0), psi(0), phid(0), thetad(0), psid(0), pretime_(this->get_clock()->now()) {
        propvel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/prop_vel", 10);
        trajectory_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/trajectory", 10, std::bind(&PIDcontroller::trajectoryCallback, this, std::placeholders::_1));
        state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/droneposition/odom", 10, std::bind(&PIDcontroller::stateCallback, this, std::placeholders::_1));
    }

private:
    void trajectoryCallback (const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 3) {
            RCLCPP_WARN(rclcpp::get_logger("PIDcontroller"), "Received incorrect number of trajectory data.");
            return;
        }
        xdes = msg->data[0];
        ydes = msg->data[1];
        zdes = msg->data[2];
    }
    void stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto &position = msg->pose.pose.position;
        const auto &orientation = msg->pose.pose.orientation;
        const auto &linear_velocity = msg->twist.twist.linear;
        const auto &angular_velocity = msg->twist.twist.angular;

        // Convert quaternion to roll, pitch, and yaw
        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        tf2::Matrix3x3 rpy(q);
        double roll, pitch, yaw;
        rpy.getRPY(roll, pitch, yaw);

        double x = position.x;
        double y = position.y;
        double z = position.z;
        double phi = roll;  // Roll angle
        double theta = pitch;  // Pitch angle
        double psi = yaw;  // Yaw angle
        double xd = linear_velocity.x;
        double yd = linear_velocity.y;
        double zd = linear_velocity.z;
        double phid = angular_velocity.x;
        double thetad = angular_velocity.y;
        double psid = angular_velocity.z;

        rclcpp::Time timenow = this->get_clock()->now();
        double dt = (timenow - pretime_).seconds();
        pretime_ = timenow;

    
        double psides = 0;
        double m = 2.11, g= 9.8;
        double U1pid = clamp(controlz_.calculate(zdes, z, dt) + 20.678, 17.5, 25.0);
        double U4pid = clamp(controlpsi_.calculate(psides, psi, dt), -0.05, 0.05);
        auto [U2pid, U3pid, phides, thetades] = controlxy_.calculatexy(xdes, ydes, x, y, phid, thetad, phi, theta, psi, dt);
        
        double U1 = U1pid;
        double U2 = U2pid;
        double U3 = U3pid;
        double U4 = U4pid;

        RCLCPP_INFO(rclcpp::get_logger("PIDcontroller"), "Control law: %f, %f, %f, %f; Frequency: %f", U1, U2, U3, U4, 1/dt);
        
        double kt = 0.00025, kd = 0.000075, l = 0.159;
        double w12, w22, w32, w42;

        w12 = (U1 * kd * l - U2 * kd - U3 * kd + U4 * kt * l) / (4 * kd * kt * l);
        w22 = (U1 * kd * l + U2 * kd - U3 * kd - U4 * kt * l) / (4 * kd * kt * l);
        w32 = (U1 * kd * l - U2 * kd + U3 * kd - U4 * kt * l) / (4 * kd * kt * l);
        w42 = (U1 * kd * l + U2 * kd + U3 * kd + U4 * kt * l) / (4 * kd * kt * l);

        // Ensure non-negative wheel speeds
        w12 = std::max(0.0, w12);
        w22 = std::max(0.0, w22);
        w32 = std::max(0.0, w32);
        w42 = std::max(0.0, w42);

        double w1 = -std::sqrt(w12);
        double w2 = std::sqrt(w22);
        double w3 = std::sqrt(w32);
        double w4 = -std::sqrt(w42);

        // Publish propeller velocities
        std_msgs::msg::Float64MultiArray prop_vel_msg;
        prop_vel_msg.data = {w1, w2, w3, w4};
        propvel_pub_->publish(prop_vel_msg);

        auto node = rclcpp::Node::make_shared("time_logger");
        rclcpp::Clock::SharedPtr clock = node->get_clock();

        auto now = clock->now();
        auto elapsed_time = now.nanoseconds(); // ROS time in nanoseconds

        auto elapsed_time_ms = elapsed_time / 1'000'000; 

        std::ofstream outfile1("datacascadedpid.txt", std::ios_base::app);
        if (outfile1.is_open()) {
            outfile1 << "Time: " << elapsed_time_ms << ", x: " << x << ", y: " << y << ", z: " << z << ", xd: " << xd << ", yd: " << yd << ", zd: " << zd << ", phi: " << phi << ", theta: " << theta << ", psi: " << psi << ", U1: " << U1 << ", U2: " << U2 << ", U3: " << U3 << ", U4: " << U4 << "\n";
            outfile1.close();
        } else {
            RCLCPP_ERROR(node->get_logger(), "Unable to open file for writing.");
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr propvel_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr trajectory_sub_; 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;    
    rclcpp::Time pretime_;
    double x, y, z, xd, yd, zd, phi, theta, psi, phid, thetad, psid;
    double xdes, ydes, zdes;
    PID controlz_;
    PID controlpsi_;
    PIDXY controlxy_;
    double prephides, prethetades, prepsides;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDcontroller>());
    rclcpp::shutdown();
    return 0;
}
