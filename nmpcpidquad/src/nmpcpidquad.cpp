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
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <fstream>


using namespace std::chrono_literals;
#define NX          ACADO_NX    // Number of differential states
#define NXA         ACADO_NXA   // Number of algebraic states
#define NU          ACADO_NU    // Number of control inputs
#define N           ACADO_N     // Number of control intervals
#define NOD         ACADO_NOD     // Number of online data
#define NY          ACADO_NY    // Number of references, nodes 0..N - 1
#define NYN         ACADO_NYN   // Number of references for node N
#define NUM_STEPS   1         // Number of simulation steps 
#define VERBOSE     1           // Show iterations: 1, Silent: 0

template <typename T>
T clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max) ? max : value;
}

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

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
          previous_error_x(0), previous_error_phi(0), previous_error_y(0), previous_error_theta(0), integralphi(0), integraltheta(0) {}

    std::tuple<double, double, double, double> calculatexy(double setpointx, double setpointy, double pvx, double pvy, double pvphi, double pvtheta, double psi, double dt) {
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

        double errorphi = phir - pvphi;
        double errortheta = thetar - pvtheta;
        double derivativephi = (errorphi - previous_error_phi) / dt;
        double derivativetheta = (errortheta - previous_error_theta) / dt;
        integralphi += errorphi * dt;
        integralphi = clamp(integralphi, -0.075, 0.075);
        integraltheta += errortheta * dt;
        integraltheta = clamp(integraltheta, -0.075, 0.075);
        previous_error_phi = errorphi;
        previous_error_theta = errortheta;

        double U2 = kpphi_ * errorphi + kdphi_ * derivativephi + kiphi_*integralphi;
        double U3 = kptheta_ * errortheta + kdtheta_ * derivativetheta + kitheta_ * integraltheta;
        U2 = clamp(U2, -0.025, 0.025);
        U3 = clamp(U3, -0.025, 0.025);

        return {U2, U3, phir, thetar};
    }

private:
    double kpx_, kdx_, kpy_, kdy_;
    double kdphi_, kpphi_, kdtheta_, kptheta_, kiphi_, kitheta_;
    double previous_error_x, previous_error_phi;
    double integralphi, integraltheta;
    double previous_error_y, previous_error_theta;
};

class NMPCPIDcontroller : public rclcpp::Node {
public:
    NMPCPIDcontroller(): Node("NMPCPIDcontroller"), controlz_(8.0, 0, 5.75), controlpsi_(3.25, 0.5, 1.75), controlxy_(1.05, 0.5275, 1.0, 1.05, 0.5275, 1.0, 0.05, 0.1, 0.05, 0.1), xdes(5.0), ydes(5.0), zdes(5.0),
    x(0), y(0), z(0), xd(0), yd(0), zd(0), phi(0), theta(0), psi(0), phid(0), thetad(0), psid(0), pretime_(this->get_clock()->now()) {
        propvel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/prop_vel", 10);
        trajectory_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/trajectory", 10, std::bind(&NMPCPIDcontroller::trajectoryCallback, this, std::placeholders::_1));
        state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/droneposition/odom", 10, std::bind(&NMPCPIDcontroller::stateCallback, this, std::placeholders::_1));
    }

private:
    void trajectoryCallback (const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 3) {
            RCLCPP_WARN(rclcpp::get_logger("NMPCPIDcontroller"), "Received incorrect number of trajectory data.");
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

        unsigned int i, iter;
        acado_timer t;

        // Reset all solver memory
        std::memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
        std::memset(&acadoVariables, 0, sizeof(acadoVariables));

        // Initialize the solver
        acado_initializeSolver();

        // Prepare a consistent initial guess
        for (i = 0; i < N + 1; ++i)
        {
            acadoVariables.x[i * NX + 0] = x;
            acadoVariables.x[i * NX + 1] = y;
            acadoVariables.x[i * NX + 2] = z;
            acadoVariables.x[i * NX + 3] = phi;
            acadoVariables.x[i * NX + 4] = theta;
            acadoVariables.x[i * NX + 5] = psi;
            acadoVariables.x[i * NX + 6] = xd;
            acadoVariables.x[i * NX + 7] = yd;
            acadoVariables.x[i * NX + 8] = zd;
            acadoVariables.x[i * NX + 9] = phid;
            acadoVariables.x[i * NX + 10] = thetad;
            acadoVariables.x[i * NX + 11] = psid;
        }

        double psides = 0;
        double m = 2.11, g= 9.8;
        double U1pid = clamp(controlz_.calculate(zdes, z, dt) + 20.678, 17.5, 25.0);
        double U4pid = clamp(controlpsi_.calculate(psides, psi, dt), -0.05, 0.05);
        auto [U2pid, U3pid, phides, thetades] = controlxy_.calculatexy(xdes, ydes, x, y, phi, theta, psi, dt);

        // Prepare references
        for (i = 0; i < N; ++i) {
            acadoVariables.y[i * NY + 0] = zdes; 
            acadoVariables.y[i * NY + 1] = phides;   
            acadoVariables.y[i * NY + 2] = thetades;   
            acadoVariables.y[i * NY + 3] = psides;
            // NMPC PID
            acadoVariables.y[i * NY + 4] = U1pid;  
            acadoVariables.y[i * NY + 5] = U2pid;
            acadoVariables.y[i * NY + 6] = U3pid;   
            acadoVariables.y[i * NY + 7] = U4pid;
            // NMPC
            //acadoVariables.y[i * NY + 4] = m*g;  
            //acadoVariables.y[i * NY + 5] = 0;
            //acadoVariables.y[i * NY + 6] = 0;   
            //acadoVariables.y[i * NY + 7] = 0;    
        }

        acadoVariables.yN[0] = zdes;  
        acadoVariables.yN[1] = phides;   
        acadoVariables.yN[2] = thetades;   
        acadoVariables.yN[3] = psides;
        for (i = 0; i < NX; ++i) {   
            acadoVariables.x0[ i ] = acadoVariables.x[NX + i];
        }

        acado_preparationStep();
        acado_feedbackStep();
        acado_shiftStates(2, 0, 0);
        acado_shiftControls(0);
        real_t* u = acado_getVariablesU();

        // Process and store control variables
        std::vector<std::vector<double>> control_variables;
        for (int i = 0; i < N; ++i) { // Loop over control intervals
            std::vector<double> control_row;
            for (int j = 0; j < NU; ++j) { // Loop over control variables per interval
                control_row.push_back(static_cast<double>(u[i * NU + j]));
            }
            control_variables.push_back(control_row);
        }

        double U1 = control_variables[0][0];
        double U2 = control_variables[0][1];
        double U3 = control_variables[0][2];
        double U4 = control_variables[0][3];

        RCLCPP_INFO(rclcpp::get_logger("NMPCPIDcontroller"), "Control law: %f, %f, %f, %f; Frequency: %f", U1, U2, U3, U4, 1/dt);

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

        //auto node = rclcpp::Node::make_shared("time_logger");
        //rclcpp::Clock::SharedPtr clock = node->get_clock();

        //auto now = clock->now();
        //auto elapsed_time = now.nanoseconds(); // ROS time in nanoseconds

        //auto elapsed_time_ms = elapsed_time / 1'000'000; 

        //std::ofstream outfile1("datasmc.txt", std::ios_base::app);
        //if (outfile1.is_open()) {
        //    outfile1 << "Time: " << elapsed_time_ms << ", x: " << x << ", y: " << y << ", z: " << z << ", xd: " << xd << ", yd: " << yd << ", zd: " << zd << ", phi: " << phi << ", theta: " << theta << ", psi: " << psi << ", U1: " << U1 << ", U2: " << U2 << ", U3: " << U3 << ", U4: " << U4 << "\n";
        //   outfile1.close();
        //} else {
        //    RCLCPP_ERROR(node->get_logger(), "Unable to open file for writing.");
        //}
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
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NMPCPIDcontroller>());
    rclcpp::shutdown();
    return 0;
}
