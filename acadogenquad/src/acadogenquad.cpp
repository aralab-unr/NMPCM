#include <acado_code_generation.hpp>
#include <iostream>  // For user input

USING_NAMESPACE_ACADO

int main()
{
    //
    // Variables
    //
    const double m = 2.11, g = 9.80;
    const double Ixx = 0.0785, Iyy = 0.0785, Izz = 0.105;

    DifferentialState x, y, z, phi, theta, psi;
    DifferentialState xd, yd, zd, phid, thetad, psid;
    Control u1, u2, u3, u4;

    DifferentialEquation f;
    f << dot(x) == xd;
    f << dot(y) == yd;
    f << dot(z) == zd;
    f << dot(phi) == phid;
    f << dot(theta) == thetad;
    f << dot(psi) == psid;
    f << dot(xd) == (1/m) * ((sin(psi) * sin(phi) + cos(psi) * sin(theta) * cos(phi)) * u1);
    f << dot(yd) == (1/m) * ((sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)) * u1);
    f << dot(zd) == (1/m) * (cos(theta) * cos(phi) * u1 - m * g);
    f << dot(phid) ==   (1/Ixx) * ( phid * thetad * (Iyy - Izz) + u2);
    f << dot(thetad) == (1/Iyy) * ( phid * psid * (Izz - Ixx) + u3);
    f << dot(psid) == (1/Izz) * ( phid * thetad * (Ixx - Iyy) + u4);;

    Function rf;
    Function rfN;

    rf << z << phi << theta << psi;
    rf << u1 << u2 << u3 << u4;

    rfN << z  << phi << theta << psi;

    DMatrix W = eye<double>(rf.getDim());
    DMatrix WN = eye<double>(rfN.getDim());

    W(0, 0) = 5.75;
    W(1, 1) = 12.5;
    W(2, 2) = 12.5;
    W(3, 3) = 12.5;
    W(4, 4) = 0.25;
    W(5, 5) = 25.75;
    W(6, 6) = 25.75;
    W(7, 7) = 25.75;

    WN(0, 0) = 0.25;
    WN(1, 1) = 0.25;
    WN(2, 2) = 0.25;
    WN(3, 3) = 0.25;

    //
    // Optimal Control Problem
    //
    const int N = 10;
    const int Ni = 10;
    const double Ts = 0.1;

    OCP ocp(0, N * Ts, N);
    ocp.subjectTo(f);

    ocp.minimizeLSQ(W, rf);
    ocp.minimizeLSQEndTerm(WN, rfN);
    ocp.subjectTo(17.5 <= u1 <= 25.0);  // Control input constraints (thrust)
    ocp.subjectTo(-0.1 <= u2 <= 0.1);  // Control input constraints (roll torque)
    ocp.subjectTo(-0.1 <= u3 <= 0.1);  // Control input constraints (pitch torque)
    ocp.subjectTo(-0.1 <= u4 <= 0.1);  // Control input constraints (yaw torque)
	
    // Adding dynamic control input constraint based on online input

    //
    // Export the code:
    //
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);

    mpc.set(INTEGRATOR_TYPE, INT_RK45);
	mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);

    mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
    mpc.set(QP_SOLVER, QP_QPOASES);
    mpc.set(HOTSTART_QP, YES);

    mpc.set(GENERATE_TEST_FILE, NO);
    mpc.set(GENERATE_MAKE_FILE, NO);
    mpc.set(GENERATE_MATLAB_INTERFACE, NO);
	

    if (mpc.exportCode("acadogen_quad") != SUCCESSFUL_RETURN)
        exit(EXIT_FAILURE);

    mpc.printDimensionsQP();

    return EXIT_SUCCESS;
}
