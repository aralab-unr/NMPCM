/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <acado_code_generation.hpp>

USING_NAMESPACE_ACADO

int main(int argc, char * const argv[ ])
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
    f << dot(phid) == (1/Ixx) * u2;
    f << dot(thetad) == (1/Iyy) * u3;
    f << dot(psid) == (1/Izz) * u4;

	Function rf;
	Function rfN;

	rf << z << phi << theta << psi << u1 << u2 << u3 << u4;
	rfN << z << phi << theta << psi;

	DMatrix W = eye<double>( rf.getDim() );
	DMatrix WN = eye<double>( rfN.getDim() );

	W(0,0) = 25.5;
	W(1,1) = 147.5;
	W(2,2) = 147.5;
	W(3,3) = 147.5;
	W(4,4) = 0.25;
	W(5,5) = 5.75;
	W(6,6) = 5.75;
	W(7,7) = 5.75;

	WN(0,0) = 0.025;
	WN(1,1) = 0.25;
	WN(2,2) = 0.25;
	WN(3,3) = 0.25;
	//
	// Optimal Control Problem
	//

	const int N  = 12;
	const int Ni = 5;
	const double Ts = 0.1;

	OCP ocp(0, N * Ts, N);

	ocp.subjectTo( f );

	ocp.minimizeLSQ(W, rf);
	ocp.minimizeLSQEndTerm(WN, rfN);

	ocp.subjectTo(15 <= u1 <= 25);  // Control input constraints (thrust)
    ocp.subjectTo(-0.01 <= u2 <= 0.01);  // Control input constraints (roll torque)
    ocp.subjectTo(-0.01 <= u3 <= 0.01);  // Control input constraints (pitch torque)
    ocp.subjectTo(-0.05 <= u4 <= 0.05);  // Control input constraints (yaw torque)

	//
	// Export the code:
	//
	OCPexport mpc( ocp );

	mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
	mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);

	mpc.set(INTEGRATOR_TYPE, INT_IRK_RIIA3);
	mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);

	mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
//	mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
	mpc.set(QP_SOLVER, QP_QPOASES);
//	mpc.set(MAX_NUM_QP_ITERATIONS, 20);
	mpc.set(HOTSTART_QP, YES);

//	mpc.set(SPARSE_QP_SOLUTION, SPARSE_SOLVER);
//	mpc.set(QP_SOLVER, QP_FORCES);

//	mpc.set(LEVENBERG_MARQUARDT, 1.0e-10);

	mpc.set(GENERATE_TEST_FILE, NO);
	mpc.set(GENERATE_MAKE_FILE, NO);
	mpc.set(GENERATE_MATLAB_INTERFACE, NO);

//	mpc.set(USE_SINGLE_PRECISION, YES);
//	mpc.set(CG_USE_OPENMP, YES);

	if (mpc.exportCode( "acadogen_quad" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}