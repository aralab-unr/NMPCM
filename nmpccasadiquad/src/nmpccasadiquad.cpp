#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <chrono>
#include <vector>
#include <casadi/casadi.hpp>
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <fstream>


using namespace std::chrono_literals;
using namespace casadi;

// Clamp function
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


constexpr double m = 2.11, g = 9.80;
constexpr double Ixx = 0.0785, Iyy = 0.0785, Izz = 0.105;

// Nonlinear dynamic equations
MX f(const MX& x, const MX& u) {
    // Extract state variables
    MX xd = x(0), yd = x(1), zd = x(2), phid = x(3), thetad = x(4), psid = x(5);
    MX x_pos = x(6), y_pos = x(7), z_pos = x(8), phi = x(9), theta = x(10), psi = x(11);
    
    // Extract control inputs
    MX U1 = u(0), U2 = u(1), U3 = u(2), U4 = u(3);

    // Trigonometric shorthand
    MX cphi = cos(phi), sphi = sin(phi);
    MX ctheta = cos(theta), stheta = sin(theta);
    MX cpsi = cos(psi), spsi = sin(psi);

    // Equations of motion
    MX xdd = (1/m) * ((spsi * sphi + cpsi * stheta * cphi) * U1 );
    MX ydd = (1/m) * ((spsi * stheta * cphi - cpsi * sphi) * U1 );
    MX zdd = (1/m) * (ctheta * cphi * U1 - m * g);
    MX phidd = (1/Ixx) * U2;
    MX thetadd = (1/Iyy) * U3;
    MX psidd = (1/Izz) * U4;

    // State derivatives
    std::vector<MX> state_derivatives = {xdd, ydd, zdd, phidd, thetadd, psidd, xd, yd, zd, phid, thetad, psid};
    MX x_dot = vertcat(state_derivatives);

    return x_dot;
}

class NMPCASADI : public rclcpp::Node {
public:
    NMPCASADI()
        : Node("NMPCASADI"), controlz_(8.0, 0, 5.75), controlpsi_(3.25, 0.5, 1.75), controlxy_(1.05, 0.5275, 1.0, 1.05, 0.5275, 1.0, 0.05, 0.1, 0.05, 0.1), xdes(5.0), ydes(5.0), zdes(5.0),
                                  x(0), y(0), z(0), xd(0), yd(0), zd(0), phi(0), theta(0), psi(0), phid(0), thetad(0), psid(0), pretime_(this->get_clock()->now()) {
        propvel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/prop_vel", 10);
        trajectory_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/trajectory", 10, std::bind(&NMPCASADI::trajectoryCallback, this, std::placeholders::_1));
        state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/droneposition/odom", 10, std::bind(&NMPCASADI::stateCallback, this, std::placeholders::_1));
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
        rclcpp::Time timenow = this->get_clock()->now();
        double dt = (timenow - pretime_).seconds();
        pretime_ = timenow;
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

        DM initialstates = DM::zeros(12, 1);
        initialstates(0) = xd;
        initialstates(1) = yd;
        initialstates(2) = zd;
        initialstates(3) = phid;
        initialstates(4) = thetad;
        initialstates(5) = psid;
        initialstates(6) = x;
        initialstates(7) = y;
        initialstates(8) = z;
        initialstates(9) = phi;
        initialstates(10) = theta;
        initialstates(11) = psi;

        int N = 10; // Number of control intervals
        double T = dt;
    
        Opti opti; // Optimization problem
        Slice all;

        // ---- decision variables ----
        MX X = opti.variable(12, N + 1); // State trajectory
        MX U = opti.variable(4, N);     // Control trajectory

        // ---- objective function ----
        MX OBJ_FUNC = 0;
        DM Q = DM::zeros(4, 4);
        Q(0, 0) = 5.75;
        Q(1, 1) = 12.5;
        Q(2, 2) = 12.5;
        Q(3, 3) = 12.5;

        DM R = DM::zeros(4, 4);
        R(0, 0) = 12.25;
        R(1, 1) = 25.75;
        R(2, 2) = 25.75;
        R(3, 3) = 25.75;

        double psides = 0;
        double U1r = clamp(controlz_.calculate(zdes, z, dt) + 20.678, 17.5, 25.0);
        double U4r = clamp(controlpsi_.calculate(psides, psi, dt), -0.05, 0.05);
        auto [U2r, U3r, phides, thetades] = controlxy_.calculatexy(xdes, ydes, x, y, phi, theta, psi, dt);

        std::vector<double> reference = {zdes, phides, thetades, psides};

        for (int k = 0; k < N; ++k) {
            // Extract the specific elements from the state matrix X
            MX st = vertcat(X(8, k), X(9, k), X(10, k), X(11, k));  
            OBJ_FUNC += mtimes(mtimes((st - reference).T(), Q), (st - reference));
        }
        std::vector<double> Uref = {m*g, 0, 0, 0};
        for (int k = 0; k < N - 1; ++k) {
            MX con = U(Slice(), k + 1) - Uref;
            OBJ_FUNC += mtimes(mtimes(con.T(), R), con);
        }

        opti.minimize(OBJ_FUNC);

        // ---- dynamic constraints ----
        for (int k = 0; k < N; ++k) {
            MX k1 = f(X(all, k), U(all, k));
            MX x_next = X(all, k) + T * k1;
            opti.subject_to(X(all, k + 1) == x_next);
        }

        // ---- path constraints ----
        // Initial constraints
        opti.subject_to(X(all, 0) == initialstates);
        // Input constraints
        opti.subject_to(17.5 <= U(0, all) <= 25.0); 
        opti.subject_to(-0.1 <= U(1, all) <= 0.1);  
        opti.subject_to(-0.1 <= U(2, all) <= 0.1);
        opti.subject_to(-0.1 <= U(3, all) <= 0.1);

        // Set solver options to suppress the output
        Dict ipopt_options;
        ipopt_options["ipopt.print_level"] = 0;    
        ipopt_options["print_time"] = 0;           
        ipopt_options["ipopt.sb"] = "yes";         
        ipopt_options["ipopt.max_iter"] = 1000;     
        ipopt_options["ipopt.tol"] = 1e-6;        
        opti.solver("ipopt", ipopt_options);

 
        auto sol = opti.solve();
        DM U_sol = sol.value(U);  // Solve and extract the control trajectory

        // Extract individual control values
        double U1 = static_cast<double>(U_sol(0).scalar());
        double U2 = static_cast<double>(U_sol(1).scalar());
        double U3 = static_cast<double>(U_sol(2).scalar());
        double U4 = static_cast<double>(U_sol(3).scalar());

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
        RCLCPP_INFO(this->get_logger(), "Casadi solution: %f, %f, %f, %f; Frequency: %f.", U1, U2, U3, U4, 1 / dt);

        //auto node = rclcpp::Node::make_shared("time_logger");
        //rclcpp::Clock::SharedPtr clock = node->get_clock();

        //auto now = clock->now();
        //auto elapsed_time = now.nanoseconds(); // ROS time in nanoseconds

        //auto elapsed_time_ms = elapsed_time / 1'000'000; 

        //std::ofstream outfile1("datampccasadi.txt", std::ios_base::app);
        //if (outfile1.is_open()) {
        //    outfile1 << "Time: " << elapsed_time_ms << ", x: " << x << ", y: " << y << ", z: " << z << ", xd: " << xd << ", yd: " << yd << ", zd: " << zd << ", phi: " << phi << ", theta: " << theta << ", psi: " << psi << ", U1: " << U1 << ", U2: " << U2 << ", U3: " << U3 << ", U4: " << U4 << "\n";
        //    outfile1.close();
        //} else {
        //    RCLCPP_ERROR(node->get_logger(), "Unable to open file for writing.");
        // }
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
    rclcpp::spin(std::make_shared<NMPCASADI>());
    rclcpp::shutdown();
    return 0;
}