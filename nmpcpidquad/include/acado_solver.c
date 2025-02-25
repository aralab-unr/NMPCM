/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
int lRun2;
ret = 0;
for (lRun1 = 0; lRun1 < 12; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 12];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 12 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 12 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 12 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 12 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 12 + 5];
acadoWorkspace.state[6] = acadoVariables.x[lRun1 * 12 + 6];
acadoWorkspace.state[7] = acadoVariables.x[lRun1 * 12 + 7];
acadoWorkspace.state[8] = acadoVariables.x[lRun1 * 12 + 8];
acadoWorkspace.state[9] = acadoVariables.x[lRun1 * 12 + 9];
acadoWorkspace.state[10] = acadoVariables.x[lRun1 * 12 + 10];
acadoWorkspace.state[11] = acadoVariables.x[lRun1 * 12 + 11];

acadoWorkspace.state[204] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.state[205] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.state[206] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.state[207] = acadoVariables.u[lRun1 * 4 + 3];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 12] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 12 + 12];
acadoWorkspace.d[lRun1 * 12 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 12 + 13];
acadoWorkspace.d[lRun1 * 12 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 12 + 14];
acadoWorkspace.d[lRun1 * 12 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 12 + 15];
acadoWorkspace.d[lRun1 * 12 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 12 + 16];
acadoWorkspace.d[lRun1 * 12 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 12 + 17];
acadoWorkspace.d[lRun1 * 12 + 6] = acadoWorkspace.state[6] - acadoVariables.x[lRun1 * 12 + 18];
acadoWorkspace.d[lRun1 * 12 + 7] = acadoWorkspace.state[7] - acadoVariables.x[lRun1 * 12 + 19];
acadoWorkspace.d[lRun1 * 12 + 8] = acadoWorkspace.state[8] - acadoVariables.x[lRun1 * 12 + 20];
acadoWorkspace.d[lRun1 * 12 + 9] = acadoWorkspace.state[9] - acadoVariables.x[lRun1 * 12 + 21];
acadoWorkspace.d[lRun1 * 12 + 10] = acadoWorkspace.state[10] - acadoVariables.x[lRun1 * 12 + 22];
acadoWorkspace.d[lRun1 * 12 + 11] = acadoWorkspace.state[11] - acadoVariables.x[lRun1 * 12 + 23];

for (lRun2 = 0; lRun2 < 144; ++lRun2)
acadoWorkspace.evGx[(0) + ((lRun2) + (lRun1 * 144))] = acadoWorkspace.state[lRun2 + 12];


acadoWorkspace.evGu[lRun1 * 48] = acadoWorkspace.state[156];
acadoWorkspace.evGu[lRun1 * 48 + 1] = acadoWorkspace.state[157];
acadoWorkspace.evGu[lRun1 * 48 + 2] = acadoWorkspace.state[158];
acadoWorkspace.evGu[lRun1 * 48 + 3] = acadoWorkspace.state[159];
acadoWorkspace.evGu[lRun1 * 48 + 4] = acadoWorkspace.state[160];
acadoWorkspace.evGu[lRun1 * 48 + 5] = acadoWorkspace.state[161];
acadoWorkspace.evGu[lRun1 * 48 + 6] = acadoWorkspace.state[162];
acadoWorkspace.evGu[lRun1 * 48 + 7] = acadoWorkspace.state[163];
acadoWorkspace.evGu[lRun1 * 48 + 8] = acadoWorkspace.state[164];
acadoWorkspace.evGu[lRun1 * 48 + 9] = acadoWorkspace.state[165];
acadoWorkspace.evGu[lRun1 * 48 + 10] = acadoWorkspace.state[166];
acadoWorkspace.evGu[lRun1 * 48 + 11] = acadoWorkspace.state[167];
acadoWorkspace.evGu[lRun1 * 48 + 12] = acadoWorkspace.state[168];
acadoWorkspace.evGu[lRun1 * 48 + 13] = acadoWorkspace.state[169];
acadoWorkspace.evGu[lRun1 * 48 + 14] = acadoWorkspace.state[170];
acadoWorkspace.evGu[lRun1 * 48 + 15] = acadoWorkspace.state[171];
acadoWorkspace.evGu[lRun1 * 48 + 16] = acadoWorkspace.state[172];
acadoWorkspace.evGu[lRun1 * 48 + 17] = acadoWorkspace.state[173];
acadoWorkspace.evGu[lRun1 * 48 + 18] = acadoWorkspace.state[174];
acadoWorkspace.evGu[lRun1 * 48 + 19] = acadoWorkspace.state[175];
acadoWorkspace.evGu[lRun1 * 48 + 20] = acadoWorkspace.state[176];
acadoWorkspace.evGu[lRun1 * 48 + 21] = acadoWorkspace.state[177];
acadoWorkspace.evGu[lRun1 * 48 + 22] = acadoWorkspace.state[178];
acadoWorkspace.evGu[lRun1 * 48 + 23] = acadoWorkspace.state[179];
acadoWorkspace.evGu[lRun1 * 48 + 24] = acadoWorkspace.state[180];
acadoWorkspace.evGu[lRun1 * 48 + 25] = acadoWorkspace.state[181];
acadoWorkspace.evGu[lRun1 * 48 + 26] = acadoWorkspace.state[182];
acadoWorkspace.evGu[lRun1 * 48 + 27] = acadoWorkspace.state[183];
acadoWorkspace.evGu[lRun1 * 48 + 28] = acadoWorkspace.state[184];
acadoWorkspace.evGu[lRun1 * 48 + 29] = acadoWorkspace.state[185];
acadoWorkspace.evGu[lRun1 * 48 + 30] = acadoWorkspace.state[186];
acadoWorkspace.evGu[lRun1 * 48 + 31] = acadoWorkspace.state[187];
acadoWorkspace.evGu[lRun1 * 48 + 32] = acadoWorkspace.state[188];
acadoWorkspace.evGu[lRun1 * 48 + 33] = acadoWorkspace.state[189];
acadoWorkspace.evGu[lRun1 * 48 + 34] = acadoWorkspace.state[190];
acadoWorkspace.evGu[lRun1 * 48 + 35] = acadoWorkspace.state[191];
acadoWorkspace.evGu[lRun1 * 48 + 36] = acadoWorkspace.state[192];
acadoWorkspace.evGu[lRun1 * 48 + 37] = acadoWorkspace.state[193];
acadoWorkspace.evGu[lRun1 * 48 + 38] = acadoWorkspace.state[194];
acadoWorkspace.evGu[lRun1 * 48 + 39] = acadoWorkspace.state[195];
acadoWorkspace.evGu[lRun1 * 48 + 40] = acadoWorkspace.state[196];
acadoWorkspace.evGu[lRun1 * 48 + 41] = acadoWorkspace.state[197];
acadoWorkspace.evGu[lRun1 * 48 + 42] = acadoWorkspace.state[198];
acadoWorkspace.evGu[lRun1 * 48 + 43] = acadoWorkspace.state[199];
acadoWorkspace.evGu[lRun1 * 48 + 44] = acadoWorkspace.state[200];
acadoWorkspace.evGu[lRun1 * 48 + 45] = acadoWorkspace.state[201];
acadoWorkspace.evGu[lRun1 * 48 + 46] = acadoWorkspace.state[202];
acadoWorkspace.evGu[lRun1 * 48 + 47] = acadoWorkspace.state[203];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 12;

/* Compute outputs: */
out[0] = xd[2];
out[1] = xd[3];
out[2] = xd[4];
out[3] = xd[5];
out[4] = u[0];
out[5] = u[1];
out[6] = u[2];
out[7] = u[3];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[2];
out[1] = xd[3];
out[2] = xd[4];
out[3] = xd[5];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 12; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 12];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 12 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 12 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 12 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 12 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 12 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 12 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 12 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 12 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[runObj * 12 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.x[runObj * 12 + 10];
acadoWorkspace.objValueIn[11] = acadoVariables.x[runObj * 12 + 11];
acadoWorkspace.objValueIn[12] = acadoVariables.u[runObj * 4];
acadoWorkspace.objValueIn[13] = acadoVariables.u[runObj * 4 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.u[runObj * 4 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.u[runObj * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 8] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 8 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 8 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 8 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 8 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 8 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 8 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 8 + 7] = acadoWorkspace.objValueOut[7];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[144];
acadoWorkspace.objValueIn[1] = acadoVariables.x[145];
acadoWorkspace.objValueIn[2] = acadoVariables.x[146];
acadoWorkspace.objValueIn[3] = acadoVariables.x[147];
acadoWorkspace.objValueIn[4] = acadoVariables.x[148];
acadoWorkspace.objValueIn[5] = acadoVariables.x[149];
acadoWorkspace.objValueIn[6] = acadoVariables.x[150];
acadoWorkspace.objValueIn[7] = acadoVariables.x[151];
acadoWorkspace.objValueIn[8] = acadoVariables.x[152];
acadoWorkspace.objValueIn[9] = acadoVariables.x[153];
acadoWorkspace.objValueIn[10] = acadoVariables.x[154];
acadoWorkspace.objValueIn[11] = acadoVariables.x[155];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7] + Gx1[8]*dOld[8] + Gx1[9]*dOld[9] + Gx1[10]*dOld[10] + Gx1[11]*dOld[11];
dNew[1] += + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3] + Gx1[16]*dOld[4] + Gx1[17]*dOld[5] + Gx1[18]*dOld[6] + Gx1[19]*dOld[7] + Gx1[20]*dOld[8] + Gx1[21]*dOld[9] + Gx1[22]*dOld[10] + Gx1[23]*dOld[11];
dNew[2] += + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5] + Gx1[30]*dOld[6] + Gx1[31]*dOld[7] + Gx1[32]*dOld[8] + Gx1[33]*dOld[9] + Gx1[34]*dOld[10] + Gx1[35]*dOld[11];
dNew[3] += + Gx1[36]*dOld[0] + Gx1[37]*dOld[1] + Gx1[38]*dOld[2] + Gx1[39]*dOld[3] + Gx1[40]*dOld[4] + Gx1[41]*dOld[5] + Gx1[42]*dOld[6] + Gx1[43]*dOld[7] + Gx1[44]*dOld[8] + Gx1[45]*dOld[9] + Gx1[46]*dOld[10] + Gx1[47]*dOld[11];
dNew[4] += + Gx1[48]*dOld[0] + Gx1[49]*dOld[1] + Gx1[50]*dOld[2] + Gx1[51]*dOld[3] + Gx1[52]*dOld[4] + Gx1[53]*dOld[5] + Gx1[54]*dOld[6] + Gx1[55]*dOld[7] + Gx1[56]*dOld[8] + Gx1[57]*dOld[9] + Gx1[58]*dOld[10] + Gx1[59]*dOld[11];
dNew[5] += + Gx1[60]*dOld[0] + Gx1[61]*dOld[1] + Gx1[62]*dOld[2] + Gx1[63]*dOld[3] + Gx1[64]*dOld[4] + Gx1[65]*dOld[5] + Gx1[66]*dOld[6] + Gx1[67]*dOld[7] + Gx1[68]*dOld[8] + Gx1[69]*dOld[9] + Gx1[70]*dOld[10] + Gx1[71]*dOld[11];
dNew[6] += + Gx1[72]*dOld[0] + Gx1[73]*dOld[1] + Gx1[74]*dOld[2] + Gx1[75]*dOld[3] + Gx1[76]*dOld[4] + Gx1[77]*dOld[5] + Gx1[78]*dOld[6] + Gx1[79]*dOld[7] + Gx1[80]*dOld[8] + Gx1[81]*dOld[9] + Gx1[82]*dOld[10] + Gx1[83]*dOld[11];
dNew[7] += + Gx1[84]*dOld[0] + Gx1[85]*dOld[1] + Gx1[86]*dOld[2] + Gx1[87]*dOld[3] + Gx1[88]*dOld[4] + Gx1[89]*dOld[5] + Gx1[90]*dOld[6] + Gx1[91]*dOld[7] + Gx1[92]*dOld[8] + Gx1[93]*dOld[9] + Gx1[94]*dOld[10] + Gx1[95]*dOld[11];
dNew[8] += + Gx1[96]*dOld[0] + Gx1[97]*dOld[1] + Gx1[98]*dOld[2] + Gx1[99]*dOld[3] + Gx1[100]*dOld[4] + Gx1[101]*dOld[5] + Gx1[102]*dOld[6] + Gx1[103]*dOld[7] + Gx1[104]*dOld[8] + Gx1[105]*dOld[9] + Gx1[106]*dOld[10] + Gx1[107]*dOld[11];
dNew[9] += + Gx1[108]*dOld[0] + Gx1[109]*dOld[1] + Gx1[110]*dOld[2] + Gx1[111]*dOld[3] + Gx1[112]*dOld[4] + Gx1[113]*dOld[5] + Gx1[114]*dOld[6] + Gx1[115]*dOld[7] + Gx1[116]*dOld[8] + Gx1[117]*dOld[9] + Gx1[118]*dOld[10] + Gx1[119]*dOld[11];
dNew[10] += + Gx1[120]*dOld[0] + Gx1[121]*dOld[1] + Gx1[122]*dOld[2] + Gx1[123]*dOld[3] + Gx1[124]*dOld[4] + Gx1[125]*dOld[5] + Gx1[126]*dOld[6] + Gx1[127]*dOld[7] + Gx1[128]*dOld[8] + Gx1[129]*dOld[9] + Gx1[130]*dOld[10] + Gx1[131]*dOld[11];
dNew[11] += + Gx1[132]*dOld[0] + Gx1[133]*dOld[1] + Gx1[134]*dOld[2] + Gx1[135]*dOld[3] + Gx1[136]*dOld[4] + Gx1[137]*dOld[5] + Gx1[138]*dOld[6] + Gx1[139]*dOld[7] + Gx1[140]*dOld[8] + Gx1[141]*dOld[9] + Gx1[142]*dOld[10] + Gx1[143]*dOld[11];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
int lRun1;
int lRun2;
for (lRun1 = 0;lRun1 < 12; ++lRun1)
for (lRun2 = 0;lRun2 < 12; ++lRun2)
Gx2[(lRun1 * 12) + (lRun2)] = Gx1[(lRun1 * 12) + (lRun2)];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[12] + Gx1[2]*Gx2[24] + Gx1[3]*Gx2[36] + Gx1[4]*Gx2[48] + Gx1[5]*Gx2[60] + Gx1[6]*Gx2[72] + Gx1[7]*Gx2[84] + Gx1[8]*Gx2[96] + Gx1[9]*Gx2[108] + Gx1[10]*Gx2[120] + Gx1[11]*Gx2[132];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[13] + Gx1[2]*Gx2[25] + Gx1[3]*Gx2[37] + Gx1[4]*Gx2[49] + Gx1[5]*Gx2[61] + Gx1[6]*Gx2[73] + Gx1[7]*Gx2[85] + Gx1[8]*Gx2[97] + Gx1[9]*Gx2[109] + Gx1[10]*Gx2[121] + Gx1[11]*Gx2[133];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[14] + Gx1[2]*Gx2[26] + Gx1[3]*Gx2[38] + Gx1[4]*Gx2[50] + Gx1[5]*Gx2[62] + Gx1[6]*Gx2[74] + Gx1[7]*Gx2[86] + Gx1[8]*Gx2[98] + Gx1[9]*Gx2[110] + Gx1[10]*Gx2[122] + Gx1[11]*Gx2[134];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[15] + Gx1[2]*Gx2[27] + Gx1[3]*Gx2[39] + Gx1[4]*Gx2[51] + Gx1[5]*Gx2[63] + Gx1[6]*Gx2[75] + Gx1[7]*Gx2[87] + Gx1[8]*Gx2[99] + Gx1[9]*Gx2[111] + Gx1[10]*Gx2[123] + Gx1[11]*Gx2[135];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[16] + Gx1[2]*Gx2[28] + Gx1[3]*Gx2[40] + Gx1[4]*Gx2[52] + Gx1[5]*Gx2[64] + Gx1[6]*Gx2[76] + Gx1[7]*Gx2[88] + Gx1[8]*Gx2[100] + Gx1[9]*Gx2[112] + Gx1[10]*Gx2[124] + Gx1[11]*Gx2[136];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[17] + Gx1[2]*Gx2[29] + Gx1[3]*Gx2[41] + Gx1[4]*Gx2[53] + Gx1[5]*Gx2[65] + Gx1[6]*Gx2[77] + Gx1[7]*Gx2[89] + Gx1[8]*Gx2[101] + Gx1[9]*Gx2[113] + Gx1[10]*Gx2[125] + Gx1[11]*Gx2[137];
Gx3[6] = + Gx1[0]*Gx2[6] + Gx1[1]*Gx2[18] + Gx1[2]*Gx2[30] + Gx1[3]*Gx2[42] + Gx1[4]*Gx2[54] + Gx1[5]*Gx2[66] + Gx1[6]*Gx2[78] + Gx1[7]*Gx2[90] + Gx1[8]*Gx2[102] + Gx1[9]*Gx2[114] + Gx1[10]*Gx2[126] + Gx1[11]*Gx2[138];
Gx3[7] = + Gx1[0]*Gx2[7] + Gx1[1]*Gx2[19] + Gx1[2]*Gx2[31] + Gx1[3]*Gx2[43] + Gx1[4]*Gx2[55] + Gx1[5]*Gx2[67] + Gx1[6]*Gx2[79] + Gx1[7]*Gx2[91] + Gx1[8]*Gx2[103] + Gx1[9]*Gx2[115] + Gx1[10]*Gx2[127] + Gx1[11]*Gx2[139];
Gx3[8] = + Gx1[0]*Gx2[8] + Gx1[1]*Gx2[20] + Gx1[2]*Gx2[32] + Gx1[3]*Gx2[44] + Gx1[4]*Gx2[56] + Gx1[5]*Gx2[68] + Gx1[6]*Gx2[80] + Gx1[7]*Gx2[92] + Gx1[8]*Gx2[104] + Gx1[9]*Gx2[116] + Gx1[10]*Gx2[128] + Gx1[11]*Gx2[140];
Gx3[9] = + Gx1[0]*Gx2[9] + Gx1[1]*Gx2[21] + Gx1[2]*Gx2[33] + Gx1[3]*Gx2[45] + Gx1[4]*Gx2[57] + Gx1[5]*Gx2[69] + Gx1[6]*Gx2[81] + Gx1[7]*Gx2[93] + Gx1[8]*Gx2[105] + Gx1[9]*Gx2[117] + Gx1[10]*Gx2[129] + Gx1[11]*Gx2[141];
Gx3[10] = + Gx1[0]*Gx2[10] + Gx1[1]*Gx2[22] + Gx1[2]*Gx2[34] + Gx1[3]*Gx2[46] + Gx1[4]*Gx2[58] + Gx1[5]*Gx2[70] + Gx1[6]*Gx2[82] + Gx1[7]*Gx2[94] + Gx1[8]*Gx2[106] + Gx1[9]*Gx2[118] + Gx1[10]*Gx2[130] + Gx1[11]*Gx2[142];
Gx3[11] = + Gx1[0]*Gx2[11] + Gx1[1]*Gx2[23] + Gx1[2]*Gx2[35] + Gx1[3]*Gx2[47] + Gx1[4]*Gx2[59] + Gx1[5]*Gx2[71] + Gx1[6]*Gx2[83] + Gx1[7]*Gx2[95] + Gx1[8]*Gx2[107] + Gx1[9]*Gx2[119] + Gx1[10]*Gx2[131] + Gx1[11]*Gx2[143];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[12] + Gx1[14]*Gx2[24] + Gx1[15]*Gx2[36] + Gx1[16]*Gx2[48] + Gx1[17]*Gx2[60] + Gx1[18]*Gx2[72] + Gx1[19]*Gx2[84] + Gx1[20]*Gx2[96] + Gx1[21]*Gx2[108] + Gx1[22]*Gx2[120] + Gx1[23]*Gx2[132];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[13] + Gx1[14]*Gx2[25] + Gx1[15]*Gx2[37] + Gx1[16]*Gx2[49] + Gx1[17]*Gx2[61] + Gx1[18]*Gx2[73] + Gx1[19]*Gx2[85] + Gx1[20]*Gx2[97] + Gx1[21]*Gx2[109] + Gx1[22]*Gx2[121] + Gx1[23]*Gx2[133];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[14] + Gx1[14]*Gx2[26] + Gx1[15]*Gx2[38] + Gx1[16]*Gx2[50] + Gx1[17]*Gx2[62] + Gx1[18]*Gx2[74] + Gx1[19]*Gx2[86] + Gx1[20]*Gx2[98] + Gx1[21]*Gx2[110] + Gx1[22]*Gx2[122] + Gx1[23]*Gx2[134];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[15] + Gx1[14]*Gx2[27] + Gx1[15]*Gx2[39] + Gx1[16]*Gx2[51] + Gx1[17]*Gx2[63] + Gx1[18]*Gx2[75] + Gx1[19]*Gx2[87] + Gx1[20]*Gx2[99] + Gx1[21]*Gx2[111] + Gx1[22]*Gx2[123] + Gx1[23]*Gx2[135];
Gx3[16] = + Gx1[12]*Gx2[4] + Gx1[13]*Gx2[16] + Gx1[14]*Gx2[28] + Gx1[15]*Gx2[40] + Gx1[16]*Gx2[52] + Gx1[17]*Gx2[64] + Gx1[18]*Gx2[76] + Gx1[19]*Gx2[88] + Gx1[20]*Gx2[100] + Gx1[21]*Gx2[112] + Gx1[22]*Gx2[124] + Gx1[23]*Gx2[136];
Gx3[17] = + Gx1[12]*Gx2[5] + Gx1[13]*Gx2[17] + Gx1[14]*Gx2[29] + Gx1[15]*Gx2[41] + Gx1[16]*Gx2[53] + Gx1[17]*Gx2[65] + Gx1[18]*Gx2[77] + Gx1[19]*Gx2[89] + Gx1[20]*Gx2[101] + Gx1[21]*Gx2[113] + Gx1[22]*Gx2[125] + Gx1[23]*Gx2[137];
Gx3[18] = + Gx1[12]*Gx2[6] + Gx1[13]*Gx2[18] + Gx1[14]*Gx2[30] + Gx1[15]*Gx2[42] + Gx1[16]*Gx2[54] + Gx1[17]*Gx2[66] + Gx1[18]*Gx2[78] + Gx1[19]*Gx2[90] + Gx1[20]*Gx2[102] + Gx1[21]*Gx2[114] + Gx1[22]*Gx2[126] + Gx1[23]*Gx2[138];
Gx3[19] = + Gx1[12]*Gx2[7] + Gx1[13]*Gx2[19] + Gx1[14]*Gx2[31] + Gx1[15]*Gx2[43] + Gx1[16]*Gx2[55] + Gx1[17]*Gx2[67] + Gx1[18]*Gx2[79] + Gx1[19]*Gx2[91] + Gx1[20]*Gx2[103] + Gx1[21]*Gx2[115] + Gx1[22]*Gx2[127] + Gx1[23]*Gx2[139];
Gx3[20] = + Gx1[12]*Gx2[8] + Gx1[13]*Gx2[20] + Gx1[14]*Gx2[32] + Gx1[15]*Gx2[44] + Gx1[16]*Gx2[56] + Gx1[17]*Gx2[68] + Gx1[18]*Gx2[80] + Gx1[19]*Gx2[92] + Gx1[20]*Gx2[104] + Gx1[21]*Gx2[116] + Gx1[22]*Gx2[128] + Gx1[23]*Gx2[140];
Gx3[21] = + Gx1[12]*Gx2[9] + Gx1[13]*Gx2[21] + Gx1[14]*Gx2[33] + Gx1[15]*Gx2[45] + Gx1[16]*Gx2[57] + Gx1[17]*Gx2[69] + Gx1[18]*Gx2[81] + Gx1[19]*Gx2[93] + Gx1[20]*Gx2[105] + Gx1[21]*Gx2[117] + Gx1[22]*Gx2[129] + Gx1[23]*Gx2[141];
Gx3[22] = + Gx1[12]*Gx2[10] + Gx1[13]*Gx2[22] + Gx1[14]*Gx2[34] + Gx1[15]*Gx2[46] + Gx1[16]*Gx2[58] + Gx1[17]*Gx2[70] + Gx1[18]*Gx2[82] + Gx1[19]*Gx2[94] + Gx1[20]*Gx2[106] + Gx1[21]*Gx2[118] + Gx1[22]*Gx2[130] + Gx1[23]*Gx2[142];
Gx3[23] = + Gx1[12]*Gx2[11] + Gx1[13]*Gx2[23] + Gx1[14]*Gx2[35] + Gx1[15]*Gx2[47] + Gx1[16]*Gx2[59] + Gx1[17]*Gx2[71] + Gx1[18]*Gx2[83] + Gx1[19]*Gx2[95] + Gx1[20]*Gx2[107] + Gx1[21]*Gx2[119] + Gx1[22]*Gx2[131] + Gx1[23]*Gx2[143];
Gx3[24] = + Gx1[24]*Gx2[0] + Gx1[25]*Gx2[12] + Gx1[26]*Gx2[24] + Gx1[27]*Gx2[36] + Gx1[28]*Gx2[48] + Gx1[29]*Gx2[60] + Gx1[30]*Gx2[72] + Gx1[31]*Gx2[84] + Gx1[32]*Gx2[96] + Gx1[33]*Gx2[108] + Gx1[34]*Gx2[120] + Gx1[35]*Gx2[132];
Gx3[25] = + Gx1[24]*Gx2[1] + Gx1[25]*Gx2[13] + Gx1[26]*Gx2[25] + Gx1[27]*Gx2[37] + Gx1[28]*Gx2[49] + Gx1[29]*Gx2[61] + Gx1[30]*Gx2[73] + Gx1[31]*Gx2[85] + Gx1[32]*Gx2[97] + Gx1[33]*Gx2[109] + Gx1[34]*Gx2[121] + Gx1[35]*Gx2[133];
Gx3[26] = + Gx1[24]*Gx2[2] + Gx1[25]*Gx2[14] + Gx1[26]*Gx2[26] + Gx1[27]*Gx2[38] + Gx1[28]*Gx2[50] + Gx1[29]*Gx2[62] + Gx1[30]*Gx2[74] + Gx1[31]*Gx2[86] + Gx1[32]*Gx2[98] + Gx1[33]*Gx2[110] + Gx1[34]*Gx2[122] + Gx1[35]*Gx2[134];
Gx3[27] = + Gx1[24]*Gx2[3] + Gx1[25]*Gx2[15] + Gx1[26]*Gx2[27] + Gx1[27]*Gx2[39] + Gx1[28]*Gx2[51] + Gx1[29]*Gx2[63] + Gx1[30]*Gx2[75] + Gx1[31]*Gx2[87] + Gx1[32]*Gx2[99] + Gx1[33]*Gx2[111] + Gx1[34]*Gx2[123] + Gx1[35]*Gx2[135];
Gx3[28] = + Gx1[24]*Gx2[4] + Gx1[25]*Gx2[16] + Gx1[26]*Gx2[28] + Gx1[27]*Gx2[40] + Gx1[28]*Gx2[52] + Gx1[29]*Gx2[64] + Gx1[30]*Gx2[76] + Gx1[31]*Gx2[88] + Gx1[32]*Gx2[100] + Gx1[33]*Gx2[112] + Gx1[34]*Gx2[124] + Gx1[35]*Gx2[136];
Gx3[29] = + Gx1[24]*Gx2[5] + Gx1[25]*Gx2[17] + Gx1[26]*Gx2[29] + Gx1[27]*Gx2[41] + Gx1[28]*Gx2[53] + Gx1[29]*Gx2[65] + Gx1[30]*Gx2[77] + Gx1[31]*Gx2[89] + Gx1[32]*Gx2[101] + Gx1[33]*Gx2[113] + Gx1[34]*Gx2[125] + Gx1[35]*Gx2[137];
Gx3[30] = + Gx1[24]*Gx2[6] + Gx1[25]*Gx2[18] + Gx1[26]*Gx2[30] + Gx1[27]*Gx2[42] + Gx1[28]*Gx2[54] + Gx1[29]*Gx2[66] + Gx1[30]*Gx2[78] + Gx1[31]*Gx2[90] + Gx1[32]*Gx2[102] + Gx1[33]*Gx2[114] + Gx1[34]*Gx2[126] + Gx1[35]*Gx2[138];
Gx3[31] = + Gx1[24]*Gx2[7] + Gx1[25]*Gx2[19] + Gx1[26]*Gx2[31] + Gx1[27]*Gx2[43] + Gx1[28]*Gx2[55] + Gx1[29]*Gx2[67] + Gx1[30]*Gx2[79] + Gx1[31]*Gx2[91] + Gx1[32]*Gx2[103] + Gx1[33]*Gx2[115] + Gx1[34]*Gx2[127] + Gx1[35]*Gx2[139];
Gx3[32] = + Gx1[24]*Gx2[8] + Gx1[25]*Gx2[20] + Gx1[26]*Gx2[32] + Gx1[27]*Gx2[44] + Gx1[28]*Gx2[56] + Gx1[29]*Gx2[68] + Gx1[30]*Gx2[80] + Gx1[31]*Gx2[92] + Gx1[32]*Gx2[104] + Gx1[33]*Gx2[116] + Gx1[34]*Gx2[128] + Gx1[35]*Gx2[140];
Gx3[33] = + Gx1[24]*Gx2[9] + Gx1[25]*Gx2[21] + Gx1[26]*Gx2[33] + Gx1[27]*Gx2[45] + Gx1[28]*Gx2[57] + Gx1[29]*Gx2[69] + Gx1[30]*Gx2[81] + Gx1[31]*Gx2[93] + Gx1[32]*Gx2[105] + Gx1[33]*Gx2[117] + Gx1[34]*Gx2[129] + Gx1[35]*Gx2[141];
Gx3[34] = + Gx1[24]*Gx2[10] + Gx1[25]*Gx2[22] + Gx1[26]*Gx2[34] + Gx1[27]*Gx2[46] + Gx1[28]*Gx2[58] + Gx1[29]*Gx2[70] + Gx1[30]*Gx2[82] + Gx1[31]*Gx2[94] + Gx1[32]*Gx2[106] + Gx1[33]*Gx2[118] + Gx1[34]*Gx2[130] + Gx1[35]*Gx2[142];
Gx3[35] = + Gx1[24]*Gx2[11] + Gx1[25]*Gx2[23] + Gx1[26]*Gx2[35] + Gx1[27]*Gx2[47] + Gx1[28]*Gx2[59] + Gx1[29]*Gx2[71] + Gx1[30]*Gx2[83] + Gx1[31]*Gx2[95] + Gx1[32]*Gx2[107] + Gx1[33]*Gx2[119] + Gx1[34]*Gx2[131] + Gx1[35]*Gx2[143];
Gx3[36] = + Gx1[36]*Gx2[0] + Gx1[37]*Gx2[12] + Gx1[38]*Gx2[24] + Gx1[39]*Gx2[36] + Gx1[40]*Gx2[48] + Gx1[41]*Gx2[60] + Gx1[42]*Gx2[72] + Gx1[43]*Gx2[84] + Gx1[44]*Gx2[96] + Gx1[45]*Gx2[108] + Gx1[46]*Gx2[120] + Gx1[47]*Gx2[132];
Gx3[37] = + Gx1[36]*Gx2[1] + Gx1[37]*Gx2[13] + Gx1[38]*Gx2[25] + Gx1[39]*Gx2[37] + Gx1[40]*Gx2[49] + Gx1[41]*Gx2[61] + Gx1[42]*Gx2[73] + Gx1[43]*Gx2[85] + Gx1[44]*Gx2[97] + Gx1[45]*Gx2[109] + Gx1[46]*Gx2[121] + Gx1[47]*Gx2[133];
Gx3[38] = + Gx1[36]*Gx2[2] + Gx1[37]*Gx2[14] + Gx1[38]*Gx2[26] + Gx1[39]*Gx2[38] + Gx1[40]*Gx2[50] + Gx1[41]*Gx2[62] + Gx1[42]*Gx2[74] + Gx1[43]*Gx2[86] + Gx1[44]*Gx2[98] + Gx1[45]*Gx2[110] + Gx1[46]*Gx2[122] + Gx1[47]*Gx2[134];
Gx3[39] = + Gx1[36]*Gx2[3] + Gx1[37]*Gx2[15] + Gx1[38]*Gx2[27] + Gx1[39]*Gx2[39] + Gx1[40]*Gx2[51] + Gx1[41]*Gx2[63] + Gx1[42]*Gx2[75] + Gx1[43]*Gx2[87] + Gx1[44]*Gx2[99] + Gx1[45]*Gx2[111] + Gx1[46]*Gx2[123] + Gx1[47]*Gx2[135];
Gx3[40] = + Gx1[36]*Gx2[4] + Gx1[37]*Gx2[16] + Gx1[38]*Gx2[28] + Gx1[39]*Gx2[40] + Gx1[40]*Gx2[52] + Gx1[41]*Gx2[64] + Gx1[42]*Gx2[76] + Gx1[43]*Gx2[88] + Gx1[44]*Gx2[100] + Gx1[45]*Gx2[112] + Gx1[46]*Gx2[124] + Gx1[47]*Gx2[136];
Gx3[41] = + Gx1[36]*Gx2[5] + Gx1[37]*Gx2[17] + Gx1[38]*Gx2[29] + Gx1[39]*Gx2[41] + Gx1[40]*Gx2[53] + Gx1[41]*Gx2[65] + Gx1[42]*Gx2[77] + Gx1[43]*Gx2[89] + Gx1[44]*Gx2[101] + Gx1[45]*Gx2[113] + Gx1[46]*Gx2[125] + Gx1[47]*Gx2[137];
Gx3[42] = + Gx1[36]*Gx2[6] + Gx1[37]*Gx2[18] + Gx1[38]*Gx2[30] + Gx1[39]*Gx2[42] + Gx1[40]*Gx2[54] + Gx1[41]*Gx2[66] + Gx1[42]*Gx2[78] + Gx1[43]*Gx2[90] + Gx1[44]*Gx2[102] + Gx1[45]*Gx2[114] + Gx1[46]*Gx2[126] + Gx1[47]*Gx2[138];
Gx3[43] = + Gx1[36]*Gx2[7] + Gx1[37]*Gx2[19] + Gx1[38]*Gx2[31] + Gx1[39]*Gx2[43] + Gx1[40]*Gx2[55] + Gx1[41]*Gx2[67] + Gx1[42]*Gx2[79] + Gx1[43]*Gx2[91] + Gx1[44]*Gx2[103] + Gx1[45]*Gx2[115] + Gx1[46]*Gx2[127] + Gx1[47]*Gx2[139];
Gx3[44] = + Gx1[36]*Gx2[8] + Gx1[37]*Gx2[20] + Gx1[38]*Gx2[32] + Gx1[39]*Gx2[44] + Gx1[40]*Gx2[56] + Gx1[41]*Gx2[68] + Gx1[42]*Gx2[80] + Gx1[43]*Gx2[92] + Gx1[44]*Gx2[104] + Gx1[45]*Gx2[116] + Gx1[46]*Gx2[128] + Gx1[47]*Gx2[140];
Gx3[45] = + Gx1[36]*Gx2[9] + Gx1[37]*Gx2[21] + Gx1[38]*Gx2[33] + Gx1[39]*Gx2[45] + Gx1[40]*Gx2[57] + Gx1[41]*Gx2[69] + Gx1[42]*Gx2[81] + Gx1[43]*Gx2[93] + Gx1[44]*Gx2[105] + Gx1[45]*Gx2[117] + Gx1[46]*Gx2[129] + Gx1[47]*Gx2[141];
Gx3[46] = + Gx1[36]*Gx2[10] + Gx1[37]*Gx2[22] + Gx1[38]*Gx2[34] + Gx1[39]*Gx2[46] + Gx1[40]*Gx2[58] + Gx1[41]*Gx2[70] + Gx1[42]*Gx2[82] + Gx1[43]*Gx2[94] + Gx1[44]*Gx2[106] + Gx1[45]*Gx2[118] + Gx1[46]*Gx2[130] + Gx1[47]*Gx2[142];
Gx3[47] = + Gx1[36]*Gx2[11] + Gx1[37]*Gx2[23] + Gx1[38]*Gx2[35] + Gx1[39]*Gx2[47] + Gx1[40]*Gx2[59] + Gx1[41]*Gx2[71] + Gx1[42]*Gx2[83] + Gx1[43]*Gx2[95] + Gx1[44]*Gx2[107] + Gx1[45]*Gx2[119] + Gx1[46]*Gx2[131] + Gx1[47]*Gx2[143];
Gx3[48] = + Gx1[48]*Gx2[0] + Gx1[49]*Gx2[12] + Gx1[50]*Gx2[24] + Gx1[51]*Gx2[36] + Gx1[52]*Gx2[48] + Gx1[53]*Gx2[60] + Gx1[54]*Gx2[72] + Gx1[55]*Gx2[84] + Gx1[56]*Gx2[96] + Gx1[57]*Gx2[108] + Gx1[58]*Gx2[120] + Gx1[59]*Gx2[132];
Gx3[49] = + Gx1[48]*Gx2[1] + Gx1[49]*Gx2[13] + Gx1[50]*Gx2[25] + Gx1[51]*Gx2[37] + Gx1[52]*Gx2[49] + Gx1[53]*Gx2[61] + Gx1[54]*Gx2[73] + Gx1[55]*Gx2[85] + Gx1[56]*Gx2[97] + Gx1[57]*Gx2[109] + Gx1[58]*Gx2[121] + Gx1[59]*Gx2[133];
Gx3[50] = + Gx1[48]*Gx2[2] + Gx1[49]*Gx2[14] + Gx1[50]*Gx2[26] + Gx1[51]*Gx2[38] + Gx1[52]*Gx2[50] + Gx1[53]*Gx2[62] + Gx1[54]*Gx2[74] + Gx1[55]*Gx2[86] + Gx1[56]*Gx2[98] + Gx1[57]*Gx2[110] + Gx1[58]*Gx2[122] + Gx1[59]*Gx2[134];
Gx3[51] = + Gx1[48]*Gx2[3] + Gx1[49]*Gx2[15] + Gx1[50]*Gx2[27] + Gx1[51]*Gx2[39] + Gx1[52]*Gx2[51] + Gx1[53]*Gx2[63] + Gx1[54]*Gx2[75] + Gx1[55]*Gx2[87] + Gx1[56]*Gx2[99] + Gx1[57]*Gx2[111] + Gx1[58]*Gx2[123] + Gx1[59]*Gx2[135];
Gx3[52] = + Gx1[48]*Gx2[4] + Gx1[49]*Gx2[16] + Gx1[50]*Gx2[28] + Gx1[51]*Gx2[40] + Gx1[52]*Gx2[52] + Gx1[53]*Gx2[64] + Gx1[54]*Gx2[76] + Gx1[55]*Gx2[88] + Gx1[56]*Gx2[100] + Gx1[57]*Gx2[112] + Gx1[58]*Gx2[124] + Gx1[59]*Gx2[136];
Gx3[53] = + Gx1[48]*Gx2[5] + Gx1[49]*Gx2[17] + Gx1[50]*Gx2[29] + Gx1[51]*Gx2[41] + Gx1[52]*Gx2[53] + Gx1[53]*Gx2[65] + Gx1[54]*Gx2[77] + Gx1[55]*Gx2[89] + Gx1[56]*Gx2[101] + Gx1[57]*Gx2[113] + Gx1[58]*Gx2[125] + Gx1[59]*Gx2[137];
Gx3[54] = + Gx1[48]*Gx2[6] + Gx1[49]*Gx2[18] + Gx1[50]*Gx2[30] + Gx1[51]*Gx2[42] + Gx1[52]*Gx2[54] + Gx1[53]*Gx2[66] + Gx1[54]*Gx2[78] + Gx1[55]*Gx2[90] + Gx1[56]*Gx2[102] + Gx1[57]*Gx2[114] + Gx1[58]*Gx2[126] + Gx1[59]*Gx2[138];
Gx3[55] = + Gx1[48]*Gx2[7] + Gx1[49]*Gx2[19] + Gx1[50]*Gx2[31] + Gx1[51]*Gx2[43] + Gx1[52]*Gx2[55] + Gx1[53]*Gx2[67] + Gx1[54]*Gx2[79] + Gx1[55]*Gx2[91] + Gx1[56]*Gx2[103] + Gx1[57]*Gx2[115] + Gx1[58]*Gx2[127] + Gx1[59]*Gx2[139];
Gx3[56] = + Gx1[48]*Gx2[8] + Gx1[49]*Gx2[20] + Gx1[50]*Gx2[32] + Gx1[51]*Gx2[44] + Gx1[52]*Gx2[56] + Gx1[53]*Gx2[68] + Gx1[54]*Gx2[80] + Gx1[55]*Gx2[92] + Gx1[56]*Gx2[104] + Gx1[57]*Gx2[116] + Gx1[58]*Gx2[128] + Gx1[59]*Gx2[140];
Gx3[57] = + Gx1[48]*Gx2[9] + Gx1[49]*Gx2[21] + Gx1[50]*Gx2[33] + Gx1[51]*Gx2[45] + Gx1[52]*Gx2[57] + Gx1[53]*Gx2[69] + Gx1[54]*Gx2[81] + Gx1[55]*Gx2[93] + Gx1[56]*Gx2[105] + Gx1[57]*Gx2[117] + Gx1[58]*Gx2[129] + Gx1[59]*Gx2[141];
Gx3[58] = + Gx1[48]*Gx2[10] + Gx1[49]*Gx2[22] + Gx1[50]*Gx2[34] + Gx1[51]*Gx2[46] + Gx1[52]*Gx2[58] + Gx1[53]*Gx2[70] + Gx1[54]*Gx2[82] + Gx1[55]*Gx2[94] + Gx1[56]*Gx2[106] + Gx1[57]*Gx2[118] + Gx1[58]*Gx2[130] + Gx1[59]*Gx2[142];
Gx3[59] = + Gx1[48]*Gx2[11] + Gx1[49]*Gx2[23] + Gx1[50]*Gx2[35] + Gx1[51]*Gx2[47] + Gx1[52]*Gx2[59] + Gx1[53]*Gx2[71] + Gx1[54]*Gx2[83] + Gx1[55]*Gx2[95] + Gx1[56]*Gx2[107] + Gx1[57]*Gx2[119] + Gx1[58]*Gx2[131] + Gx1[59]*Gx2[143];
Gx3[60] = + Gx1[60]*Gx2[0] + Gx1[61]*Gx2[12] + Gx1[62]*Gx2[24] + Gx1[63]*Gx2[36] + Gx1[64]*Gx2[48] + Gx1[65]*Gx2[60] + Gx1[66]*Gx2[72] + Gx1[67]*Gx2[84] + Gx1[68]*Gx2[96] + Gx1[69]*Gx2[108] + Gx1[70]*Gx2[120] + Gx1[71]*Gx2[132];
Gx3[61] = + Gx1[60]*Gx2[1] + Gx1[61]*Gx2[13] + Gx1[62]*Gx2[25] + Gx1[63]*Gx2[37] + Gx1[64]*Gx2[49] + Gx1[65]*Gx2[61] + Gx1[66]*Gx2[73] + Gx1[67]*Gx2[85] + Gx1[68]*Gx2[97] + Gx1[69]*Gx2[109] + Gx1[70]*Gx2[121] + Gx1[71]*Gx2[133];
Gx3[62] = + Gx1[60]*Gx2[2] + Gx1[61]*Gx2[14] + Gx1[62]*Gx2[26] + Gx1[63]*Gx2[38] + Gx1[64]*Gx2[50] + Gx1[65]*Gx2[62] + Gx1[66]*Gx2[74] + Gx1[67]*Gx2[86] + Gx1[68]*Gx2[98] + Gx1[69]*Gx2[110] + Gx1[70]*Gx2[122] + Gx1[71]*Gx2[134];
Gx3[63] = + Gx1[60]*Gx2[3] + Gx1[61]*Gx2[15] + Gx1[62]*Gx2[27] + Gx1[63]*Gx2[39] + Gx1[64]*Gx2[51] + Gx1[65]*Gx2[63] + Gx1[66]*Gx2[75] + Gx1[67]*Gx2[87] + Gx1[68]*Gx2[99] + Gx1[69]*Gx2[111] + Gx1[70]*Gx2[123] + Gx1[71]*Gx2[135];
Gx3[64] = + Gx1[60]*Gx2[4] + Gx1[61]*Gx2[16] + Gx1[62]*Gx2[28] + Gx1[63]*Gx2[40] + Gx1[64]*Gx2[52] + Gx1[65]*Gx2[64] + Gx1[66]*Gx2[76] + Gx1[67]*Gx2[88] + Gx1[68]*Gx2[100] + Gx1[69]*Gx2[112] + Gx1[70]*Gx2[124] + Gx1[71]*Gx2[136];
Gx3[65] = + Gx1[60]*Gx2[5] + Gx1[61]*Gx2[17] + Gx1[62]*Gx2[29] + Gx1[63]*Gx2[41] + Gx1[64]*Gx2[53] + Gx1[65]*Gx2[65] + Gx1[66]*Gx2[77] + Gx1[67]*Gx2[89] + Gx1[68]*Gx2[101] + Gx1[69]*Gx2[113] + Gx1[70]*Gx2[125] + Gx1[71]*Gx2[137];
Gx3[66] = + Gx1[60]*Gx2[6] + Gx1[61]*Gx2[18] + Gx1[62]*Gx2[30] + Gx1[63]*Gx2[42] + Gx1[64]*Gx2[54] + Gx1[65]*Gx2[66] + Gx1[66]*Gx2[78] + Gx1[67]*Gx2[90] + Gx1[68]*Gx2[102] + Gx1[69]*Gx2[114] + Gx1[70]*Gx2[126] + Gx1[71]*Gx2[138];
Gx3[67] = + Gx1[60]*Gx2[7] + Gx1[61]*Gx2[19] + Gx1[62]*Gx2[31] + Gx1[63]*Gx2[43] + Gx1[64]*Gx2[55] + Gx1[65]*Gx2[67] + Gx1[66]*Gx2[79] + Gx1[67]*Gx2[91] + Gx1[68]*Gx2[103] + Gx1[69]*Gx2[115] + Gx1[70]*Gx2[127] + Gx1[71]*Gx2[139];
Gx3[68] = + Gx1[60]*Gx2[8] + Gx1[61]*Gx2[20] + Gx1[62]*Gx2[32] + Gx1[63]*Gx2[44] + Gx1[64]*Gx2[56] + Gx1[65]*Gx2[68] + Gx1[66]*Gx2[80] + Gx1[67]*Gx2[92] + Gx1[68]*Gx2[104] + Gx1[69]*Gx2[116] + Gx1[70]*Gx2[128] + Gx1[71]*Gx2[140];
Gx3[69] = + Gx1[60]*Gx2[9] + Gx1[61]*Gx2[21] + Gx1[62]*Gx2[33] + Gx1[63]*Gx2[45] + Gx1[64]*Gx2[57] + Gx1[65]*Gx2[69] + Gx1[66]*Gx2[81] + Gx1[67]*Gx2[93] + Gx1[68]*Gx2[105] + Gx1[69]*Gx2[117] + Gx1[70]*Gx2[129] + Gx1[71]*Gx2[141];
Gx3[70] = + Gx1[60]*Gx2[10] + Gx1[61]*Gx2[22] + Gx1[62]*Gx2[34] + Gx1[63]*Gx2[46] + Gx1[64]*Gx2[58] + Gx1[65]*Gx2[70] + Gx1[66]*Gx2[82] + Gx1[67]*Gx2[94] + Gx1[68]*Gx2[106] + Gx1[69]*Gx2[118] + Gx1[70]*Gx2[130] + Gx1[71]*Gx2[142];
Gx3[71] = + Gx1[60]*Gx2[11] + Gx1[61]*Gx2[23] + Gx1[62]*Gx2[35] + Gx1[63]*Gx2[47] + Gx1[64]*Gx2[59] + Gx1[65]*Gx2[71] + Gx1[66]*Gx2[83] + Gx1[67]*Gx2[95] + Gx1[68]*Gx2[107] + Gx1[69]*Gx2[119] + Gx1[70]*Gx2[131] + Gx1[71]*Gx2[143];
Gx3[72] = + Gx1[72]*Gx2[0] + Gx1[73]*Gx2[12] + Gx1[74]*Gx2[24] + Gx1[75]*Gx2[36] + Gx1[76]*Gx2[48] + Gx1[77]*Gx2[60] + Gx1[78]*Gx2[72] + Gx1[79]*Gx2[84] + Gx1[80]*Gx2[96] + Gx1[81]*Gx2[108] + Gx1[82]*Gx2[120] + Gx1[83]*Gx2[132];
Gx3[73] = + Gx1[72]*Gx2[1] + Gx1[73]*Gx2[13] + Gx1[74]*Gx2[25] + Gx1[75]*Gx2[37] + Gx1[76]*Gx2[49] + Gx1[77]*Gx2[61] + Gx1[78]*Gx2[73] + Gx1[79]*Gx2[85] + Gx1[80]*Gx2[97] + Gx1[81]*Gx2[109] + Gx1[82]*Gx2[121] + Gx1[83]*Gx2[133];
Gx3[74] = + Gx1[72]*Gx2[2] + Gx1[73]*Gx2[14] + Gx1[74]*Gx2[26] + Gx1[75]*Gx2[38] + Gx1[76]*Gx2[50] + Gx1[77]*Gx2[62] + Gx1[78]*Gx2[74] + Gx1[79]*Gx2[86] + Gx1[80]*Gx2[98] + Gx1[81]*Gx2[110] + Gx1[82]*Gx2[122] + Gx1[83]*Gx2[134];
Gx3[75] = + Gx1[72]*Gx2[3] + Gx1[73]*Gx2[15] + Gx1[74]*Gx2[27] + Gx1[75]*Gx2[39] + Gx1[76]*Gx2[51] + Gx1[77]*Gx2[63] + Gx1[78]*Gx2[75] + Gx1[79]*Gx2[87] + Gx1[80]*Gx2[99] + Gx1[81]*Gx2[111] + Gx1[82]*Gx2[123] + Gx1[83]*Gx2[135];
Gx3[76] = + Gx1[72]*Gx2[4] + Gx1[73]*Gx2[16] + Gx1[74]*Gx2[28] + Gx1[75]*Gx2[40] + Gx1[76]*Gx2[52] + Gx1[77]*Gx2[64] + Gx1[78]*Gx2[76] + Gx1[79]*Gx2[88] + Gx1[80]*Gx2[100] + Gx1[81]*Gx2[112] + Gx1[82]*Gx2[124] + Gx1[83]*Gx2[136];
Gx3[77] = + Gx1[72]*Gx2[5] + Gx1[73]*Gx2[17] + Gx1[74]*Gx2[29] + Gx1[75]*Gx2[41] + Gx1[76]*Gx2[53] + Gx1[77]*Gx2[65] + Gx1[78]*Gx2[77] + Gx1[79]*Gx2[89] + Gx1[80]*Gx2[101] + Gx1[81]*Gx2[113] + Gx1[82]*Gx2[125] + Gx1[83]*Gx2[137];
Gx3[78] = + Gx1[72]*Gx2[6] + Gx1[73]*Gx2[18] + Gx1[74]*Gx2[30] + Gx1[75]*Gx2[42] + Gx1[76]*Gx2[54] + Gx1[77]*Gx2[66] + Gx1[78]*Gx2[78] + Gx1[79]*Gx2[90] + Gx1[80]*Gx2[102] + Gx1[81]*Gx2[114] + Gx1[82]*Gx2[126] + Gx1[83]*Gx2[138];
Gx3[79] = + Gx1[72]*Gx2[7] + Gx1[73]*Gx2[19] + Gx1[74]*Gx2[31] + Gx1[75]*Gx2[43] + Gx1[76]*Gx2[55] + Gx1[77]*Gx2[67] + Gx1[78]*Gx2[79] + Gx1[79]*Gx2[91] + Gx1[80]*Gx2[103] + Gx1[81]*Gx2[115] + Gx1[82]*Gx2[127] + Gx1[83]*Gx2[139];
Gx3[80] = + Gx1[72]*Gx2[8] + Gx1[73]*Gx2[20] + Gx1[74]*Gx2[32] + Gx1[75]*Gx2[44] + Gx1[76]*Gx2[56] + Gx1[77]*Gx2[68] + Gx1[78]*Gx2[80] + Gx1[79]*Gx2[92] + Gx1[80]*Gx2[104] + Gx1[81]*Gx2[116] + Gx1[82]*Gx2[128] + Gx1[83]*Gx2[140];
Gx3[81] = + Gx1[72]*Gx2[9] + Gx1[73]*Gx2[21] + Gx1[74]*Gx2[33] + Gx1[75]*Gx2[45] + Gx1[76]*Gx2[57] + Gx1[77]*Gx2[69] + Gx1[78]*Gx2[81] + Gx1[79]*Gx2[93] + Gx1[80]*Gx2[105] + Gx1[81]*Gx2[117] + Gx1[82]*Gx2[129] + Gx1[83]*Gx2[141];
Gx3[82] = + Gx1[72]*Gx2[10] + Gx1[73]*Gx2[22] + Gx1[74]*Gx2[34] + Gx1[75]*Gx2[46] + Gx1[76]*Gx2[58] + Gx1[77]*Gx2[70] + Gx1[78]*Gx2[82] + Gx1[79]*Gx2[94] + Gx1[80]*Gx2[106] + Gx1[81]*Gx2[118] + Gx1[82]*Gx2[130] + Gx1[83]*Gx2[142];
Gx3[83] = + Gx1[72]*Gx2[11] + Gx1[73]*Gx2[23] + Gx1[74]*Gx2[35] + Gx1[75]*Gx2[47] + Gx1[76]*Gx2[59] + Gx1[77]*Gx2[71] + Gx1[78]*Gx2[83] + Gx1[79]*Gx2[95] + Gx1[80]*Gx2[107] + Gx1[81]*Gx2[119] + Gx1[82]*Gx2[131] + Gx1[83]*Gx2[143];
Gx3[84] = + Gx1[84]*Gx2[0] + Gx1[85]*Gx2[12] + Gx1[86]*Gx2[24] + Gx1[87]*Gx2[36] + Gx1[88]*Gx2[48] + Gx1[89]*Gx2[60] + Gx1[90]*Gx2[72] + Gx1[91]*Gx2[84] + Gx1[92]*Gx2[96] + Gx1[93]*Gx2[108] + Gx1[94]*Gx2[120] + Gx1[95]*Gx2[132];
Gx3[85] = + Gx1[84]*Gx2[1] + Gx1[85]*Gx2[13] + Gx1[86]*Gx2[25] + Gx1[87]*Gx2[37] + Gx1[88]*Gx2[49] + Gx1[89]*Gx2[61] + Gx1[90]*Gx2[73] + Gx1[91]*Gx2[85] + Gx1[92]*Gx2[97] + Gx1[93]*Gx2[109] + Gx1[94]*Gx2[121] + Gx1[95]*Gx2[133];
Gx3[86] = + Gx1[84]*Gx2[2] + Gx1[85]*Gx2[14] + Gx1[86]*Gx2[26] + Gx1[87]*Gx2[38] + Gx1[88]*Gx2[50] + Gx1[89]*Gx2[62] + Gx1[90]*Gx2[74] + Gx1[91]*Gx2[86] + Gx1[92]*Gx2[98] + Gx1[93]*Gx2[110] + Gx1[94]*Gx2[122] + Gx1[95]*Gx2[134];
Gx3[87] = + Gx1[84]*Gx2[3] + Gx1[85]*Gx2[15] + Gx1[86]*Gx2[27] + Gx1[87]*Gx2[39] + Gx1[88]*Gx2[51] + Gx1[89]*Gx2[63] + Gx1[90]*Gx2[75] + Gx1[91]*Gx2[87] + Gx1[92]*Gx2[99] + Gx1[93]*Gx2[111] + Gx1[94]*Gx2[123] + Gx1[95]*Gx2[135];
Gx3[88] = + Gx1[84]*Gx2[4] + Gx1[85]*Gx2[16] + Gx1[86]*Gx2[28] + Gx1[87]*Gx2[40] + Gx1[88]*Gx2[52] + Gx1[89]*Gx2[64] + Gx1[90]*Gx2[76] + Gx1[91]*Gx2[88] + Gx1[92]*Gx2[100] + Gx1[93]*Gx2[112] + Gx1[94]*Gx2[124] + Gx1[95]*Gx2[136];
Gx3[89] = + Gx1[84]*Gx2[5] + Gx1[85]*Gx2[17] + Gx1[86]*Gx2[29] + Gx1[87]*Gx2[41] + Gx1[88]*Gx2[53] + Gx1[89]*Gx2[65] + Gx1[90]*Gx2[77] + Gx1[91]*Gx2[89] + Gx1[92]*Gx2[101] + Gx1[93]*Gx2[113] + Gx1[94]*Gx2[125] + Gx1[95]*Gx2[137];
Gx3[90] = + Gx1[84]*Gx2[6] + Gx1[85]*Gx2[18] + Gx1[86]*Gx2[30] + Gx1[87]*Gx2[42] + Gx1[88]*Gx2[54] + Gx1[89]*Gx2[66] + Gx1[90]*Gx2[78] + Gx1[91]*Gx2[90] + Gx1[92]*Gx2[102] + Gx1[93]*Gx2[114] + Gx1[94]*Gx2[126] + Gx1[95]*Gx2[138];
Gx3[91] = + Gx1[84]*Gx2[7] + Gx1[85]*Gx2[19] + Gx1[86]*Gx2[31] + Gx1[87]*Gx2[43] + Gx1[88]*Gx2[55] + Gx1[89]*Gx2[67] + Gx1[90]*Gx2[79] + Gx1[91]*Gx2[91] + Gx1[92]*Gx2[103] + Gx1[93]*Gx2[115] + Gx1[94]*Gx2[127] + Gx1[95]*Gx2[139];
Gx3[92] = + Gx1[84]*Gx2[8] + Gx1[85]*Gx2[20] + Gx1[86]*Gx2[32] + Gx1[87]*Gx2[44] + Gx1[88]*Gx2[56] + Gx1[89]*Gx2[68] + Gx1[90]*Gx2[80] + Gx1[91]*Gx2[92] + Gx1[92]*Gx2[104] + Gx1[93]*Gx2[116] + Gx1[94]*Gx2[128] + Gx1[95]*Gx2[140];
Gx3[93] = + Gx1[84]*Gx2[9] + Gx1[85]*Gx2[21] + Gx1[86]*Gx2[33] + Gx1[87]*Gx2[45] + Gx1[88]*Gx2[57] + Gx1[89]*Gx2[69] + Gx1[90]*Gx2[81] + Gx1[91]*Gx2[93] + Gx1[92]*Gx2[105] + Gx1[93]*Gx2[117] + Gx1[94]*Gx2[129] + Gx1[95]*Gx2[141];
Gx3[94] = + Gx1[84]*Gx2[10] + Gx1[85]*Gx2[22] + Gx1[86]*Gx2[34] + Gx1[87]*Gx2[46] + Gx1[88]*Gx2[58] + Gx1[89]*Gx2[70] + Gx1[90]*Gx2[82] + Gx1[91]*Gx2[94] + Gx1[92]*Gx2[106] + Gx1[93]*Gx2[118] + Gx1[94]*Gx2[130] + Gx1[95]*Gx2[142];
Gx3[95] = + Gx1[84]*Gx2[11] + Gx1[85]*Gx2[23] + Gx1[86]*Gx2[35] + Gx1[87]*Gx2[47] + Gx1[88]*Gx2[59] + Gx1[89]*Gx2[71] + Gx1[90]*Gx2[83] + Gx1[91]*Gx2[95] + Gx1[92]*Gx2[107] + Gx1[93]*Gx2[119] + Gx1[94]*Gx2[131] + Gx1[95]*Gx2[143];
Gx3[96] = + Gx1[96]*Gx2[0] + Gx1[97]*Gx2[12] + Gx1[98]*Gx2[24] + Gx1[99]*Gx2[36] + Gx1[100]*Gx2[48] + Gx1[101]*Gx2[60] + Gx1[102]*Gx2[72] + Gx1[103]*Gx2[84] + Gx1[104]*Gx2[96] + Gx1[105]*Gx2[108] + Gx1[106]*Gx2[120] + Gx1[107]*Gx2[132];
Gx3[97] = + Gx1[96]*Gx2[1] + Gx1[97]*Gx2[13] + Gx1[98]*Gx2[25] + Gx1[99]*Gx2[37] + Gx1[100]*Gx2[49] + Gx1[101]*Gx2[61] + Gx1[102]*Gx2[73] + Gx1[103]*Gx2[85] + Gx1[104]*Gx2[97] + Gx1[105]*Gx2[109] + Gx1[106]*Gx2[121] + Gx1[107]*Gx2[133];
Gx3[98] = + Gx1[96]*Gx2[2] + Gx1[97]*Gx2[14] + Gx1[98]*Gx2[26] + Gx1[99]*Gx2[38] + Gx1[100]*Gx2[50] + Gx1[101]*Gx2[62] + Gx1[102]*Gx2[74] + Gx1[103]*Gx2[86] + Gx1[104]*Gx2[98] + Gx1[105]*Gx2[110] + Gx1[106]*Gx2[122] + Gx1[107]*Gx2[134];
Gx3[99] = + Gx1[96]*Gx2[3] + Gx1[97]*Gx2[15] + Gx1[98]*Gx2[27] + Gx1[99]*Gx2[39] + Gx1[100]*Gx2[51] + Gx1[101]*Gx2[63] + Gx1[102]*Gx2[75] + Gx1[103]*Gx2[87] + Gx1[104]*Gx2[99] + Gx1[105]*Gx2[111] + Gx1[106]*Gx2[123] + Gx1[107]*Gx2[135];
Gx3[100] = + Gx1[96]*Gx2[4] + Gx1[97]*Gx2[16] + Gx1[98]*Gx2[28] + Gx1[99]*Gx2[40] + Gx1[100]*Gx2[52] + Gx1[101]*Gx2[64] + Gx1[102]*Gx2[76] + Gx1[103]*Gx2[88] + Gx1[104]*Gx2[100] + Gx1[105]*Gx2[112] + Gx1[106]*Gx2[124] + Gx1[107]*Gx2[136];
Gx3[101] = + Gx1[96]*Gx2[5] + Gx1[97]*Gx2[17] + Gx1[98]*Gx2[29] + Gx1[99]*Gx2[41] + Gx1[100]*Gx2[53] + Gx1[101]*Gx2[65] + Gx1[102]*Gx2[77] + Gx1[103]*Gx2[89] + Gx1[104]*Gx2[101] + Gx1[105]*Gx2[113] + Gx1[106]*Gx2[125] + Gx1[107]*Gx2[137];
Gx3[102] = + Gx1[96]*Gx2[6] + Gx1[97]*Gx2[18] + Gx1[98]*Gx2[30] + Gx1[99]*Gx2[42] + Gx1[100]*Gx2[54] + Gx1[101]*Gx2[66] + Gx1[102]*Gx2[78] + Gx1[103]*Gx2[90] + Gx1[104]*Gx2[102] + Gx1[105]*Gx2[114] + Gx1[106]*Gx2[126] + Gx1[107]*Gx2[138];
Gx3[103] = + Gx1[96]*Gx2[7] + Gx1[97]*Gx2[19] + Gx1[98]*Gx2[31] + Gx1[99]*Gx2[43] + Gx1[100]*Gx2[55] + Gx1[101]*Gx2[67] + Gx1[102]*Gx2[79] + Gx1[103]*Gx2[91] + Gx1[104]*Gx2[103] + Gx1[105]*Gx2[115] + Gx1[106]*Gx2[127] + Gx1[107]*Gx2[139];
Gx3[104] = + Gx1[96]*Gx2[8] + Gx1[97]*Gx2[20] + Gx1[98]*Gx2[32] + Gx1[99]*Gx2[44] + Gx1[100]*Gx2[56] + Gx1[101]*Gx2[68] + Gx1[102]*Gx2[80] + Gx1[103]*Gx2[92] + Gx1[104]*Gx2[104] + Gx1[105]*Gx2[116] + Gx1[106]*Gx2[128] + Gx1[107]*Gx2[140];
Gx3[105] = + Gx1[96]*Gx2[9] + Gx1[97]*Gx2[21] + Gx1[98]*Gx2[33] + Gx1[99]*Gx2[45] + Gx1[100]*Gx2[57] + Gx1[101]*Gx2[69] + Gx1[102]*Gx2[81] + Gx1[103]*Gx2[93] + Gx1[104]*Gx2[105] + Gx1[105]*Gx2[117] + Gx1[106]*Gx2[129] + Gx1[107]*Gx2[141];
Gx3[106] = + Gx1[96]*Gx2[10] + Gx1[97]*Gx2[22] + Gx1[98]*Gx2[34] + Gx1[99]*Gx2[46] + Gx1[100]*Gx2[58] + Gx1[101]*Gx2[70] + Gx1[102]*Gx2[82] + Gx1[103]*Gx2[94] + Gx1[104]*Gx2[106] + Gx1[105]*Gx2[118] + Gx1[106]*Gx2[130] + Gx1[107]*Gx2[142];
Gx3[107] = + Gx1[96]*Gx2[11] + Gx1[97]*Gx2[23] + Gx1[98]*Gx2[35] + Gx1[99]*Gx2[47] + Gx1[100]*Gx2[59] + Gx1[101]*Gx2[71] + Gx1[102]*Gx2[83] + Gx1[103]*Gx2[95] + Gx1[104]*Gx2[107] + Gx1[105]*Gx2[119] + Gx1[106]*Gx2[131] + Gx1[107]*Gx2[143];
Gx3[108] = + Gx1[108]*Gx2[0] + Gx1[109]*Gx2[12] + Gx1[110]*Gx2[24] + Gx1[111]*Gx2[36] + Gx1[112]*Gx2[48] + Gx1[113]*Gx2[60] + Gx1[114]*Gx2[72] + Gx1[115]*Gx2[84] + Gx1[116]*Gx2[96] + Gx1[117]*Gx2[108] + Gx1[118]*Gx2[120] + Gx1[119]*Gx2[132];
Gx3[109] = + Gx1[108]*Gx2[1] + Gx1[109]*Gx2[13] + Gx1[110]*Gx2[25] + Gx1[111]*Gx2[37] + Gx1[112]*Gx2[49] + Gx1[113]*Gx2[61] + Gx1[114]*Gx2[73] + Gx1[115]*Gx2[85] + Gx1[116]*Gx2[97] + Gx1[117]*Gx2[109] + Gx1[118]*Gx2[121] + Gx1[119]*Gx2[133];
Gx3[110] = + Gx1[108]*Gx2[2] + Gx1[109]*Gx2[14] + Gx1[110]*Gx2[26] + Gx1[111]*Gx2[38] + Gx1[112]*Gx2[50] + Gx1[113]*Gx2[62] + Gx1[114]*Gx2[74] + Gx1[115]*Gx2[86] + Gx1[116]*Gx2[98] + Gx1[117]*Gx2[110] + Gx1[118]*Gx2[122] + Gx1[119]*Gx2[134];
Gx3[111] = + Gx1[108]*Gx2[3] + Gx1[109]*Gx2[15] + Gx1[110]*Gx2[27] + Gx1[111]*Gx2[39] + Gx1[112]*Gx2[51] + Gx1[113]*Gx2[63] + Gx1[114]*Gx2[75] + Gx1[115]*Gx2[87] + Gx1[116]*Gx2[99] + Gx1[117]*Gx2[111] + Gx1[118]*Gx2[123] + Gx1[119]*Gx2[135];
Gx3[112] = + Gx1[108]*Gx2[4] + Gx1[109]*Gx2[16] + Gx1[110]*Gx2[28] + Gx1[111]*Gx2[40] + Gx1[112]*Gx2[52] + Gx1[113]*Gx2[64] + Gx1[114]*Gx2[76] + Gx1[115]*Gx2[88] + Gx1[116]*Gx2[100] + Gx1[117]*Gx2[112] + Gx1[118]*Gx2[124] + Gx1[119]*Gx2[136];
Gx3[113] = + Gx1[108]*Gx2[5] + Gx1[109]*Gx2[17] + Gx1[110]*Gx2[29] + Gx1[111]*Gx2[41] + Gx1[112]*Gx2[53] + Gx1[113]*Gx2[65] + Gx1[114]*Gx2[77] + Gx1[115]*Gx2[89] + Gx1[116]*Gx2[101] + Gx1[117]*Gx2[113] + Gx1[118]*Gx2[125] + Gx1[119]*Gx2[137];
Gx3[114] = + Gx1[108]*Gx2[6] + Gx1[109]*Gx2[18] + Gx1[110]*Gx2[30] + Gx1[111]*Gx2[42] + Gx1[112]*Gx2[54] + Gx1[113]*Gx2[66] + Gx1[114]*Gx2[78] + Gx1[115]*Gx2[90] + Gx1[116]*Gx2[102] + Gx1[117]*Gx2[114] + Gx1[118]*Gx2[126] + Gx1[119]*Gx2[138];
Gx3[115] = + Gx1[108]*Gx2[7] + Gx1[109]*Gx2[19] + Gx1[110]*Gx2[31] + Gx1[111]*Gx2[43] + Gx1[112]*Gx2[55] + Gx1[113]*Gx2[67] + Gx1[114]*Gx2[79] + Gx1[115]*Gx2[91] + Gx1[116]*Gx2[103] + Gx1[117]*Gx2[115] + Gx1[118]*Gx2[127] + Gx1[119]*Gx2[139];
Gx3[116] = + Gx1[108]*Gx2[8] + Gx1[109]*Gx2[20] + Gx1[110]*Gx2[32] + Gx1[111]*Gx2[44] + Gx1[112]*Gx2[56] + Gx1[113]*Gx2[68] + Gx1[114]*Gx2[80] + Gx1[115]*Gx2[92] + Gx1[116]*Gx2[104] + Gx1[117]*Gx2[116] + Gx1[118]*Gx2[128] + Gx1[119]*Gx2[140];
Gx3[117] = + Gx1[108]*Gx2[9] + Gx1[109]*Gx2[21] + Gx1[110]*Gx2[33] + Gx1[111]*Gx2[45] + Gx1[112]*Gx2[57] + Gx1[113]*Gx2[69] + Gx1[114]*Gx2[81] + Gx1[115]*Gx2[93] + Gx1[116]*Gx2[105] + Gx1[117]*Gx2[117] + Gx1[118]*Gx2[129] + Gx1[119]*Gx2[141];
Gx3[118] = + Gx1[108]*Gx2[10] + Gx1[109]*Gx2[22] + Gx1[110]*Gx2[34] + Gx1[111]*Gx2[46] + Gx1[112]*Gx2[58] + Gx1[113]*Gx2[70] + Gx1[114]*Gx2[82] + Gx1[115]*Gx2[94] + Gx1[116]*Gx2[106] + Gx1[117]*Gx2[118] + Gx1[118]*Gx2[130] + Gx1[119]*Gx2[142];
Gx3[119] = + Gx1[108]*Gx2[11] + Gx1[109]*Gx2[23] + Gx1[110]*Gx2[35] + Gx1[111]*Gx2[47] + Gx1[112]*Gx2[59] + Gx1[113]*Gx2[71] + Gx1[114]*Gx2[83] + Gx1[115]*Gx2[95] + Gx1[116]*Gx2[107] + Gx1[117]*Gx2[119] + Gx1[118]*Gx2[131] + Gx1[119]*Gx2[143];
Gx3[120] = + Gx1[120]*Gx2[0] + Gx1[121]*Gx2[12] + Gx1[122]*Gx2[24] + Gx1[123]*Gx2[36] + Gx1[124]*Gx2[48] + Gx1[125]*Gx2[60] + Gx1[126]*Gx2[72] + Gx1[127]*Gx2[84] + Gx1[128]*Gx2[96] + Gx1[129]*Gx2[108] + Gx1[130]*Gx2[120] + Gx1[131]*Gx2[132];
Gx3[121] = + Gx1[120]*Gx2[1] + Gx1[121]*Gx2[13] + Gx1[122]*Gx2[25] + Gx1[123]*Gx2[37] + Gx1[124]*Gx2[49] + Gx1[125]*Gx2[61] + Gx1[126]*Gx2[73] + Gx1[127]*Gx2[85] + Gx1[128]*Gx2[97] + Gx1[129]*Gx2[109] + Gx1[130]*Gx2[121] + Gx1[131]*Gx2[133];
Gx3[122] = + Gx1[120]*Gx2[2] + Gx1[121]*Gx2[14] + Gx1[122]*Gx2[26] + Gx1[123]*Gx2[38] + Gx1[124]*Gx2[50] + Gx1[125]*Gx2[62] + Gx1[126]*Gx2[74] + Gx1[127]*Gx2[86] + Gx1[128]*Gx2[98] + Gx1[129]*Gx2[110] + Gx1[130]*Gx2[122] + Gx1[131]*Gx2[134];
Gx3[123] = + Gx1[120]*Gx2[3] + Gx1[121]*Gx2[15] + Gx1[122]*Gx2[27] + Gx1[123]*Gx2[39] + Gx1[124]*Gx2[51] + Gx1[125]*Gx2[63] + Gx1[126]*Gx2[75] + Gx1[127]*Gx2[87] + Gx1[128]*Gx2[99] + Gx1[129]*Gx2[111] + Gx1[130]*Gx2[123] + Gx1[131]*Gx2[135];
Gx3[124] = + Gx1[120]*Gx2[4] + Gx1[121]*Gx2[16] + Gx1[122]*Gx2[28] + Gx1[123]*Gx2[40] + Gx1[124]*Gx2[52] + Gx1[125]*Gx2[64] + Gx1[126]*Gx2[76] + Gx1[127]*Gx2[88] + Gx1[128]*Gx2[100] + Gx1[129]*Gx2[112] + Gx1[130]*Gx2[124] + Gx1[131]*Gx2[136];
Gx3[125] = + Gx1[120]*Gx2[5] + Gx1[121]*Gx2[17] + Gx1[122]*Gx2[29] + Gx1[123]*Gx2[41] + Gx1[124]*Gx2[53] + Gx1[125]*Gx2[65] + Gx1[126]*Gx2[77] + Gx1[127]*Gx2[89] + Gx1[128]*Gx2[101] + Gx1[129]*Gx2[113] + Gx1[130]*Gx2[125] + Gx1[131]*Gx2[137];
Gx3[126] = + Gx1[120]*Gx2[6] + Gx1[121]*Gx2[18] + Gx1[122]*Gx2[30] + Gx1[123]*Gx2[42] + Gx1[124]*Gx2[54] + Gx1[125]*Gx2[66] + Gx1[126]*Gx2[78] + Gx1[127]*Gx2[90] + Gx1[128]*Gx2[102] + Gx1[129]*Gx2[114] + Gx1[130]*Gx2[126] + Gx1[131]*Gx2[138];
Gx3[127] = + Gx1[120]*Gx2[7] + Gx1[121]*Gx2[19] + Gx1[122]*Gx2[31] + Gx1[123]*Gx2[43] + Gx1[124]*Gx2[55] + Gx1[125]*Gx2[67] + Gx1[126]*Gx2[79] + Gx1[127]*Gx2[91] + Gx1[128]*Gx2[103] + Gx1[129]*Gx2[115] + Gx1[130]*Gx2[127] + Gx1[131]*Gx2[139];
Gx3[128] = + Gx1[120]*Gx2[8] + Gx1[121]*Gx2[20] + Gx1[122]*Gx2[32] + Gx1[123]*Gx2[44] + Gx1[124]*Gx2[56] + Gx1[125]*Gx2[68] + Gx1[126]*Gx2[80] + Gx1[127]*Gx2[92] + Gx1[128]*Gx2[104] + Gx1[129]*Gx2[116] + Gx1[130]*Gx2[128] + Gx1[131]*Gx2[140];
Gx3[129] = + Gx1[120]*Gx2[9] + Gx1[121]*Gx2[21] + Gx1[122]*Gx2[33] + Gx1[123]*Gx2[45] + Gx1[124]*Gx2[57] + Gx1[125]*Gx2[69] + Gx1[126]*Gx2[81] + Gx1[127]*Gx2[93] + Gx1[128]*Gx2[105] + Gx1[129]*Gx2[117] + Gx1[130]*Gx2[129] + Gx1[131]*Gx2[141];
Gx3[130] = + Gx1[120]*Gx2[10] + Gx1[121]*Gx2[22] + Gx1[122]*Gx2[34] + Gx1[123]*Gx2[46] + Gx1[124]*Gx2[58] + Gx1[125]*Gx2[70] + Gx1[126]*Gx2[82] + Gx1[127]*Gx2[94] + Gx1[128]*Gx2[106] + Gx1[129]*Gx2[118] + Gx1[130]*Gx2[130] + Gx1[131]*Gx2[142];
Gx3[131] = + Gx1[120]*Gx2[11] + Gx1[121]*Gx2[23] + Gx1[122]*Gx2[35] + Gx1[123]*Gx2[47] + Gx1[124]*Gx2[59] + Gx1[125]*Gx2[71] + Gx1[126]*Gx2[83] + Gx1[127]*Gx2[95] + Gx1[128]*Gx2[107] + Gx1[129]*Gx2[119] + Gx1[130]*Gx2[131] + Gx1[131]*Gx2[143];
Gx3[132] = + Gx1[132]*Gx2[0] + Gx1[133]*Gx2[12] + Gx1[134]*Gx2[24] + Gx1[135]*Gx2[36] + Gx1[136]*Gx2[48] + Gx1[137]*Gx2[60] + Gx1[138]*Gx2[72] + Gx1[139]*Gx2[84] + Gx1[140]*Gx2[96] + Gx1[141]*Gx2[108] + Gx1[142]*Gx2[120] + Gx1[143]*Gx2[132];
Gx3[133] = + Gx1[132]*Gx2[1] + Gx1[133]*Gx2[13] + Gx1[134]*Gx2[25] + Gx1[135]*Gx2[37] + Gx1[136]*Gx2[49] + Gx1[137]*Gx2[61] + Gx1[138]*Gx2[73] + Gx1[139]*Gx2[85] + Gx1[140]*Gx2[97] + Gx1[141]*Gx2[109] + Gx1[142]*Gx2[121] + Gx1[143]*Gx2[133];
Gx3[134] = + Gx1[132]*Gx2[2] + Gx1[133]*Gx2[14] + Gx1[134]*Gx2[26] + Gx1[135]*Gx2[38] + Gx1[136]*Gx2[50] + Gx1[137]*Gx2[62] + Gx1[138]*Gx2[74] + Gx1[139]*Gx2[86] + Gx1[140]*Gx2[98] + Gx1[141]*Gx2[110] + Gx1[142]*Gx2[122] + Gx1[143]*Gx2[134];
Gx3[135] = + Gx1[132]*Gx2[3] + Gx1[133]*Gx2[15] + Gx1[134]*Gx2[27] + Gx1[135]*Gx2[39] + Gx1[136]*Gx2[51] + Gx1[137]*Gx2[63] + Gx1[138]*Gx2[75] + Gx1[139]*Gx2[87] + Gx1[140]*Gx2[99] + Gx1[141]*Gx2[111] + Gx1[142]*Gx2[123] + Gx1[143]*Gx2[135];
Gx3[136] = + Gx1[132]*Gx2[4] + Gx1[133]*Gx2[16] + Gx1[134]*Gx2[28] + Gx1[135]*Gx2[40] + Gx1[136]*Gx2[52] + Gx1[137]*Gx2[64] + Gx1[138]*Gx2[76] + Gx1[139]*Gx2[88] + Gx1[140]*Gx2[100] + Gx1[141]*Gx2[112] + Gx1[142]*Gx2[124] + Gx1[143]*Gx2[136];
Gx3[137] = + Gx1[132]*Gx2[5] + Gx1[133]*Gx2[17] + Gx1[134]*Gx2[29] + Gx1[135]*Gx2[41] + Gx1[136]*Gx2[53] + Gx1[137]*Gx2[65] + Gx1[138]*Gx2[77] + Gx1[139]*Gx2[89] + Gx1[140]*Gx2[101] + Gx1[141]*Gx2[113] + Gx1[142]*Gx2[125] + Gx1[143]*Gx2[137];
Gx3[138] = + Gx1[132]*Gx2[6] + Gx1[133]*Gx2[18] + Gx1[134]*Gx2[30] + Gx1[135]*Gx2[42] + Gx1[136]*Gx2[54] + Gx1[137]*Gx2[66] + Gx1[138]*Gx2[78] + Gx1[139]*Gx2[90] + Gx1[140]*Gx2[102] + Gx1[141]*Gx2[114] + Gx1[142]*Gx2[126] + Gx1[143]*Gx2[138];
Gx3[139] = + Gx1[132]*Gx2[7] + Gx1[133]*Gx2[19] + Gx1[134]*Gx2[31] + Gx1[135]*Gx2[43] + Gx1[136]*Gx2[55] + Gx1[137]*Gx2[67] + Gx1[138]*Gx2[79] + Gx1[139]*Gx2[91] + Gx1[140]*Gx2[103] + Gx1[141]*Gx2[115] + Gx1[142]*Gx2[127] + Gx1[143]*Gx2[139];
Gx3[140] = + Gx1[132]*Gx2[8] + Gx1[133]*Gx2[20] + Gx1[134]*Gx2[32] + Gx1[135]*Gx2[44] + Gx1[136]*Gx2[56] + Gx1[137]*Gx2[68] + Gx1[138]*Gx2[80] + Gx1[139]*Gx2[92] + Gx1[140]*Gx2[104] + Gx1[141]*Gx2[116] + Gx1[142]*Gx2[128] + Gx1[143]*Gx2[140];
Gx3[141] = + Gx1[132]*Gx2[9] + Gx1[133]*Gx2[21] + Gx1[134]*Gx2[33] + Gx1[135]*Gx2[45] + Gx1[136]*Gx2[57] + Gx1[137]*Gx2[69] + Gx1[138]*Gx2[81] + Gx1[139]*Gx2[93] + Gx1[140]*Gx2[105] + Gx1[141]*Gx2[117] + Gx1[142]*Gx2[129] + Gx1[143]*Gx2[141];
Gx3[142] = + Gx1[132]*Gx2[10] + Gx1[133]*Gx2[22] + Gx1[134]*Gx2[34] + Gx1[135]*Gx2[46] + Gx1[136]*Gx2[58] + Gx1[137]*Gx2[70] + Gx1[138]*Gx2[82] + Gx1[139]*Gx2[94] + Gx1[140]*Gx2[106] + Gx1[141]*Gx2[118] + Gx1[142]*Gx2[130] + Gx1[143]*Gx2[142];
Gx3[143] = + Gx1[132]*Gx2[11] + Gx1[133]*Gx2[23] + Gx1[134]*Gx2[35] + Gx1[135]*Gx2[47] + Gx1[136]*Gx2[59] + Gx1[137]*Gx2[71] + Gx1[138]*Gx2[83] + Gx1[139]*Gx2[95] + Gx1[140]*Gx2[107] + Gx1[141]*Gx2[119] + Gx1[142]*Gx2[131] + Gx1[143]*Gx2[143];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[12] + Gx1[4]*Gu1[16] + Gx1[5]*Gu1[20] + Gx1[6]*Gu1[24] + Gx1[7]*Gu1[28] + Gx1[8]*Gu1[32] + Gx1[9]*Gu1[36] + Gx1[10]*Gu1[40] + Gx1[11]*Gu1[44];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[9] + Gx1[3]*Gu1[13] + Gx1[4]*Gu1[17] + Gx1[5]*Gu1[21] + Gx1[6]*Gu1[25] + Gx1[7]*Gu1[29] + Gx1[8]*Gu1[33] + Gx1[9]*Gu1[37] + Gx1[10]*Gu1[41] + Gx1[11]*Gu1[45];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[10] + Gx1[3]*Gu1[14] + Gx1[4]*Gu1[18] + Gx1[5]*Gu1[22] + Gx1[6]*Gu1[26] + Gx1[7]*Gu1[30] + Gx1[8]*Gu1[34] + Gx1[9]*Gu1[38] + Gx1[10]*Gu1[42] + Gx1[11]*Gu1[46];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[11] + Gx1[3]*Gu1[15] + Gx1[4]*Gu1[19] + Gx1[5]*Gu1[23] + Gx1[6]*Gu1[27] + Gx1[7]*Gu1[31] + Gx1[8]*Gu1[35] + Gx1[9]*Gu1[39] + Gx1[10]*Gu1[43] + Gx1[11]*Gu1[47];
Gu2[4] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[4] + Gx1[14]*Gu1[8] + Gx1[15]*Gu1[12] + Gx1[16]*Gu1[16] + Gx1[17]*Gu1[20] + Gx1[18]*Gu1[24] + Gx1[19]*Gu1[28] + Gx1[20]*Gu1[32] + Gx1[21]*Gu1[36] + Gx1[22]*Gu1[40] + Gx1[23]*Gu1[44];
Gu2[5] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[5] + Gx1[14]*Gu1[9] + Gx1[15]*Gu1[13] + Gx1[16]*Gu1[17] + Gx1[17]*Gu1[21] + Gx1[18]*Gu1[25] + Gx1[19]*Gu1[29] + Gx1[20]*Gu1[33] + Gx1[21]*Gu1[37] + Gx1[22]*Gu1[41] + Gx1[23]*Gu1[45];
Gu2[6] = + Gx1[12]*Gu1[2] + Gx1[13]*Gu1[6] + Gx1[14]*Gu1[10] + Gx1[15]*Gu1[14] + Gx1[16]*Gu1[18] + Gx1[17]*Gu1[22] + Gx1[18]*Gu1[26] + Gx1[19]*Gu1[30] + Gx1[20]*Gu1[34] + Gx1[21]*Gu1[38] + Gx1[22]*Gu1[42] + Gx1[23]*Gu1[46];
Gu2[7] = + Gx1[12]*Gu1[3] + Gx1[13]*Gu1[7] + Gx1[14]*Gu1[11] + Gx1[15]*Gu1[15] + Gx1[16]*Gu1[19] + Gx1[17]*Gu1[23] + Gx1[18]*Gu1[27] + Gx1[19]*Gu1[31] + Gx1[20]*Gu1[35] + Gx1[21]*Gu1[39] + Gx1[22]*Gu1[43] + Gx1[23]*Gu1[47];
Gu2[8] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[4] + Gx1[26]*Gu1[8] + Gx1[27]*Gu1[12] + Gx1[28]*Gu1[16] + Gx1[29]*Gu1[20] + Gx1[30]*Gu1[24] + Gx1[31]*Gu1[28] + Gx1[32]*Gu1[32] + Gx1[33]*Gu1[36] + Gx1[34]*Gu1[40] + Gx1[35]*Gu1[44];
Gu2[9] = + Gx1[24]*Gu1[1] + Gx1[25]*Gu1[5] + Gx1[26]*Gu1[9] + Gx1[27]*Gu1[13] + Gx1[28]*Gu1[17] + Gx1[29]*Gu1[21] + Gx1[30]*Gu1[25] + Gx1[31]*Gu1[29] + Gx1[32]*Gu1[33] + Gx1[33]*Gu1[37] + Gx1[34]*Gu1[41] + Gx1[35]*Gu1[45];
Gu2[10] = + Gx1[24]*Gu1[2] + Gx1[25]*Gu1[6] + Gx1[26]*Gu1[10] + Gx1[27]*Gu1[14] + Gx1[28]*Gu1[18] + Gx1[29]*Gu1[22] + Gx1[30]*Gu1[26] + Gx1[31]*Gu1[30] + Gx1[32]*Gu1[34] + Gx1[33]*Gu1[38] + Gx1[34]*Gu1[42] + Gx1[35]*Gu1[46];
Gu2[11] = + Gx1[24]*Gu1[3] + Gx1[25]*Gu1[7] + Gx1[26]*Gu1[11] + Gx1[27]*Gu1[15] + Gx1[28]*Gu1[19] + Gx1[29]*Gu1[23] + Gx1[30]*Gu1[27] + Gx1[31]*Gu1[31] + Gx1[32]*Gu1[35] + Gx1[33]*Gu1[39] + Gx1[34]*Gu1[43] + Gx1[35]*Gu1[47];
Gu2[12] = + Gx1[36]*Gu1[0] + Gx1[37]*Gu1[4] + Gx1[38]*Gu1[8] + Gx1[39]*Gu1[12] + Gx1[40]*Gu1[16] + Gx1[41]*Gu1[20] + Gx1[42]*Gu1[24] + Gx1[43]*Gu1[28] + Gx1[44]*Gu1[32] + Gx1[45]*Gu1[36] + Gx1[46]*Gu1[40] + Gx1[47]*Gu1[44];
Gu2[13] = + Gx1[36]*Gu1[1] + Gx1[37]*Gu1[5] + Gx1[38]*Gu1[9] + Gx1[39]*Gu1[13] + Gx1[40]*Gu1[17] + Gx1[41]*Gu1[21] + Gx1[42]*Gu1[25] + Gx1[43]*Gu1[29] + Gx1[44]*Gu1[33] + Gx1[45]*Gu1[37] + Gx1[46]*Gu1[41] + Gx1[47]*Gu1[45];
Gu2[14] = + Gx1[36]*Gu1[2] + Gx1[37]*Gu1[6] + Gx1[38]*Gu1[10] + Gx1[39]*Gu1[14] + Gx1[40]*Gu1[18] + Gx1[41]*Gu1[22] + Gx1[42]*Gu1[26] + Gx1[43]*Gu1[30] + Gx1[44]*Gu1[34] + Gx1[45]*Gu1[38] + Gx1[46]*Gu1[42] + Gx1[47]*Gu1[46];
Gu2[15] = + Gx1[36]*Gu1[3] + Gx1[37]*Gu1[7] + Gx1[38]*Gu1[11] + Gx1[39]*Gu1[15] + Gx1[40]*Gu1[19] + Gx1[41]*Gu1[23] + Gx1[42]*Gu1[27] + Gx1[43]*Gu1[31] + Gx1[44]*Gu1[35] + Gx1[45]*Gu1[39] + Gx1[46]*Gu1[43] + Gx1[47]*Gu1[47];
Gu2[16] = + Gx1[48]*Gu1[0] + Gx1[49]*Gu1[4] + Gx1[50]*Gu1[8] + Gx1[51]*Gu1[12] + Gx1[52]*Gu1[16] + Gx1[53]*Gu1[20] + Gx1[54]*Gu1[24] + Gx1[55]*Gu1[28] + Gx1[56]*Gu1[32] + Gx1[57]*Gu1[36] + Gx1[58]*Gu1[40] + Gx1[59]*Gu1[44];
Gu2[17] = + Gx1[48]*Gu1[1] + Gx1[49]*Gu1[5] + Gx1[50]*Gu1[9] + Gx1[51]*Gu1[13] + Gx1[52]*Gu1[17] + Gx1[53]*Gu1[21] + Gx1[54]*Gu1[25] + Gx1[55]*Gu1[29] + Gx1[56]*Gu1[33] + Gx1[57]*Gu1[37] + Gx1[58]*Gu1[41] + Gx1[59]*Gu1[45];
Gu2[18] = + Gx1[48]*Gu1[2] + Gx1[49]*Gu1[6] + Gx1[50]*Gu1[10] + Gx1[51]*Gu1[14] + Gx1[52]*Gu1[18] + Gx1[53]*Gu1[22] + Gx1[54]*Gu1[26] + Gx1[55]*Gu1[30] + Gx1[56]*Gu1[34] + Gx1[57]*Gu1[38] + Gx1[58]*Gu1[42] + Gx1[59]*Gu1[46];
Gu2[19] = + Gx1[48]*Gu1[3] + Gx1[49]*Gu1[7] + Gx1[50]*Gu1[11] + Gx1[51]*Gu1[15] + Gx1[52]*Gu1[19] + Gx1[53]*Gu1[23] + Gx1[54]*Gu1[27] + Gx1[55]*Gu1[31] + Gx1[56]*Gu1[35] + Gx1[57]*Gu1[39] + Gx1[58]*Gu1[43] + Gx1[59]*Gu1[47];
Gu2[20] = + Gx1[60]*Gu1[0] + Gx1[61]*Gu1[4] + Gx1[62]*Gu1[8] + Gx1[63]*Gu1[12] + Gx1[64]*Gu1[16] + Gx1[65]*Gu1[20] + Gx1[66]*Gu1[24] + Gx1[67]*Gu1[28] + Gx1[68]*Gu1[32] + Gx1[69]*Gu1[36] + Gx1[70]*Gu1[40] + Gx1[71]*Gu1[44];
Gu2[21] = + Gx1[60]*Gu1[1] + Gx1[61]*Gu1[5] + Gx1[62]*Gu1[9] + Gx1[63]*Gu1[13] + Gx1[64]*Gu1[17] + Gx1[65]*Gu1[21] + Gx1[66]*Gu1[25] + Gx1[67]*Gu1[29] + Gx1[68]*Gu1[33] + Gx1[69]*Gu1[37] + Gx1[70]*Gu1[41] + Gx1[71]*Gu1[45];
Gu2[22] = + Gx1[60]*Gu1[2] + Gx1[61]*Gu1[6] + Gx1[62]*Gu1[10] + Gx1[63]*Gu1[14] + Gx1[64]*Gu1[18] + Gx1[65]*Gu1[22] + Gx1[66]*Gu1[26] + Gx1[67]*Gu1[30] + Gx1[68]*Gu1[34] + Gx1[69]*Gu1[38] + Gx1[70]*Gu1[42] + Gx1[71]*Gu1[46];
Gu2[23] = + Gx1[60]*Gu1[3] + Gx1[61]*Gu1[7] + Gx1[62]*Gu1[11] + Gx1[63]*Gu1[15] + Gx1[64]*Gu1[19] + Gx1[65]*Gu1[23] + Gx1[66]*Gu1[27] + Gx1[67]*Gu1[31] + Gx1[68]*Gu1[35] + Gx1[69]*Gu1[39] + Gx1[70]*Gu1[43] + Gx1[71]*Gu1[47];
Gu2[24] = + Gx1[72]*Gu1[0] + Gx1[73]*Gu1[4] + Gx1[74]*Gu1[8] + Gx1[75]*Gu1[12] + Gx1[76]*Gu1[16] + Gx1[77]*Gu1[20] + Gx1[78]*Gu1[24] + Gx1[79]*Gu1[28] + Gx1[80]*Gu1[32] + Gx1[81]*Gu1[36] + Gx1[82]*Gu1[40] + Gx1[83]*Gu1[44];
Gu2[25] = + Gx1[72]*Gu1[1] + Gx1[73]*Gu1[5] + Gx1[74]*Gu1[9] + Gx1[75]*Gu1[13] + Gx1[76]*Gu1[17] + Gx1[77]*Gu1[21] + Gx1[78]*Gu1[25] + Gx1[79]*Gu1[29] + Gx1[80]*Gu1[33] + Gx1[81]*Gu1[37] + Gx1[82]*Gu1[41] + Gx1[83]*Gu1[45];
Gu2[26] = + Gx1[72]*Gu1[2] + Gx1[73]*Gu1[6] + Gx1[74]*Gu1[10] + Gx1[75]*Gu1[14] + Gx1[76]*Gu1[18] + Gx1[77]*Gu1[22] + Gx1[78]*Gu1[26] + Gx1[79]*Gu1[30] + Gx1[80]*Gu1[34] + Gx1[81]*Gu1[38] + Gx1[82]*Gu1[42] + Gx1[83]*Gu1[46];
Gu2[27] = + Gx1[72]*Gu1[3] + Gx1[73]*Gu1[7] + Gx1[74]*Gu1[11] + Gx1[75]*Gu1[15] + Gx1[76]*Gu1[19] + Gx1[77]*Gu1[23] + Gx1[78]*Gu1[27] + Gx1[79]*Gu1[31] + Gx1[80]*Gu1[35] + Gx1[81]*Gu1[39] + Gx1[82]*Gu1[43] + Gx1[83]*Gu1[47];
Gu2[28] = + Gx1[84]*Gu1[0] + Gx1[85]*Gu1[4] + Gx1[86]*Gu1[8] + Gx1[87]*Gu1[12] + Gx1[88]*Gu1[16] + Gx1[89]*Gu1[20] + Gx1[90]*Gu1[24] + Gx1[91]*Gu1[28] + Gx1[92]*Gu1[32] + Gx1[93]*Gu1[36] + Gx1[94]*Gu1[40] + Gx1[95]*Gu1[44];
Gu2[29] = + Gx1[84]*Gu1[1] + Gx1[85]*Gu1[5] + Gx1[86]*Gu1[9] + Gx1[87]*Gu1[13] + Gx1[88]*Gu1[17] + Gx1[89]*Gu1[21] + Gx1[90]*Gu1[25] + Gx1[91]*Gu1[29] + Gx1[92]*Gu1[33] + Gx1[93]*Gu1[37] + Gx1[94]*Gu1[41] + Gx1[95]*Gu1[45];
Gu2[30] = + Gx1[84]*Gu1[2] + Gx1[85]*Gu1[6] + Gx1[86]*Gu1[10] + Gx1[87]*Gu1[14] + Gx1[88]*Gu1[18] + Gx1[89]*Gu1[22] + Gx1[90]*Gu1[26] + Gx1[91]*Gu1[30] + Gx1[92]*Gu1[34] + Gx1[93]*Gu1[38] + Gx1[94]*Gu1[42] + Gx1[95]*Gu1[46];
Gu2[31] = + Gx1[84]*Gu1[3] + Gx1[85]*Gu1[7] + Gx1[86]*Gu1[11] + Gx1[87]*Gu1[15] + Gx1[88]*Gu1[19] + Gx1[89]*Gu1[23] + Gx1[90]*Gu1[27] + Gx1[91]*Gu1[31] + Gx1[92]*Gu1[35] + Gx1[93]*Gu1[39] + Gx1[94]*Gu1[43] + Gx1[95]*Gu1[47];
Gu2[32] = + Gx1[96]*Gu1[0] + Gx1[97]*Gu1[4] + Gx1[98]*Gu1[8] + Gx1[99]*Gu1[12] + Gx1[100]*Gu1[16] + Gx1[101]*Gu1[20] + Gx1[102]*Gu1[24] + Gx1[103]*Gu1[28] + Gx1[104]*Gu1[32] + Gx1[105]*Gu1[36] + Gx1[106]*Gu1[40] + Gx1[107]*Gu1[44];
Gu2[33] = + Gx1[96]*Gu1[1] + Gx1[97]*Gu1[5] + Gx1[98]*Gu1[9] + Gx1[99]*Gu1[13] + Gx1[100]*Gu1[17] + Gx1[101]*Gu1[21] + Gx1[102]*Gu1[25] + Gx1[103]*Gu1[29] + Gx1[104]*Gu1[33] + Gx1[105]*Gu1[37] + Gx1[106]*Gu1[41] + Gx1[107]*Gu1[45];
Gu2[34] = + Gx1[96]*Gu1[2] + Gx1[97]*Gu1[6] + Gx1[98]*Gu1[10] + Gx1[99]*Gu1[14] + Gx1[100]*Gu1[18] + Gx1[101]*Gu1[22] + Gx1[102]*Gu1[26] + Gx1[103]*Gu1[30] + Gx1[104]*Gu1[34] + Gx1[105]*Gu1[38] + Gx1[106]*Gu1[42] + Gx1[107]*Gu1[46];
Gu2[35] = + Gx1[96]*Gu1[3] + Gx1[97]*Gu1[7] + Gx1[98]*Gu1[11] + Gx1[99]*Gu1[15] + Gx1[100]*Gu1[19] + Gx1[101]*Gu1[23] + Gx1[102]*Gu1[27] + Gx1[103]*Gu1[31] + Gx1[104]*Gu1[35] + Gx1[105]*Gu1[39] + Gx1[106]*Gu1[43] + Gx1[107]*Gu1[47];
Gu2[36] = + Gx1[108]*Gu1[0] + Gx1[109]*Gu1[4] + Gx1[110]*Gu1[8] + Gx1[111]*Gu1[12] + Gx1[112]*Gu1[16] + Gx1[113]*Gu1[20] + Gx1[114]*Gu1[24] + Gx1[115]*Gu1[28] + Gx1[116]*Gu1[32] + Gx1[117]*Gu1[36] + Gx1[118]*Gu1[40] + Gx1[119]*Gu1[44];
Gu2[37] = + Gx1[108]*Gu1[1] + Gx1[109]*Gu1[5] + Gx1[110]*Gu1[9] + Gx1[111]*Gu1[13] + Gx1[112]*Gu1[17] + Gx1[113]*Gu1[21] + Gx1[114]*Gu1[25] + Gx1[115]*Gu1[29] + Gx1[116]*Gu1[33] + Gx1[117]*Gu1[37] + Gx1[118]*Gu1[41] + Gx1[119]*Gu1[45];
Gu2[38] = + Gx1[108]*Gu1[2] + Gx1[109]*Gu1[6] + Gx1[110]*Gu1[10] + Gx1[111]*Gu1[14] + Gx1[112]*Gu1[18] + Gx1[113]*Gu1[22] + Gx1[114]*Gu1[26] + Gx1[115]*Gu1[30] + Gx1[116]*Gu1[34] + Gx1[117]*Gu1[38] + Gx1[118]*Gu1[42] + Gx1[119]*Gu1[46];
Gu2[39] = + Gx1[108]*Gu1[3] + Gx1[109]*Gu1[7] + Gx1[110]*Gu1[11] + Gx1[111]*Gu1[15] + Gx1[112]*Gu1[19] + Gx1[113]*Gu1[23] + Gx1[114]*Gu1[27] + Gx1[115]*Gu1[31] + Gx1[116]*Gu1[35] + Gx1[117]*Gu1[39] + Gx1[118]*Gu1[43] + Gx1[119]*Gu1[47];
Gu2[40] = + Gx1[120]*Gu1[0] + Gx1[121]*Gu1[4] + Gx1[122]*Gu1[8] + Gx1[123]*Gu1[12] + Gx1[124]*Gu1[16] + Gx1[125]*Gu1[20] + Gx1[126]*Gu1[24] + Gx1[127]*Gu1[28] + Gx1[128]*Gu1[32] + Gx1[129]*Gu1[36] + Gx1[130]*Gu1[40] + Gx1[131]*Gu1[44];
Gu2[41] = + Gx1[120]*Gu1[1] + Gx1[121]*Gu1[5] + Gx1[122]*Gu1[9] + Gx1[123]*Gu1[13] + Gx1[124]*Gu1[17] + Gx1[125]*Gu1[21] + Gx1[126]*Gu1[25] + Gx1[127]*Gu1[29] + Gx1[128]*Gu1[33] + Gx1[129]*Gu1[37] + Gx1[130]*Gu1[41] + Gx1[131]*Gu1[45];
Gu2[42] = + Gx1[120]*Gu1[2] + Gx1[121]*Gu1[6] + Gx1[122]*Gu1[10] + Gx1[123]*Gu1[14] + Gx1[124]*Gu1[18] + Gx1[125]*Gu1[22] + Gx1[126]*Gu1[26] + Gx1[127]*Gu1[30] + Gx1[128]*Gu1[34] + Gx1[129]*Gu1[38] + Gx1[130]*Gu1[42] + Gx1[131]*Gu1[46];
Gu2[43] = + Gx1[120]*Gu1[3] + Gx1[121]*Gu1[7] + Gx1[122]*Gu1[11] + Gx1[123]*Gu1[15] + Gx1[124]*Gu1[19] + Gx1[125]*Gu1[23] + Gx1[126]*Gu1[27] + Gx1[127]*Gu1[31] + Gx1[128]*Gu1[35] + Gx1[129]*Gu1[39] + Gx1[130]*Gu1[43] + Gx1[131]*Gu1[47];
Gu2[44] = + Gx1[132]*Gu1[0] + Gx1[133]*Gu1[4] + Gx1[134]*Gu1[8] + Gx1[135]*Gu1[12] + Gx1[136]*Gu1[16] + Gx1[137]*Gu1[20] + Gx1[138]*Gu1[24] + Gx1[139]*Gu1[28] + Gx1[140]*Gu1[32] + Gx1[141]*Gu1[36] + Gx1[142]*Gu1[40] + Gx1[143]*Gu1[44];
Gu2[45] = + Gx1[132]*Gu1[1] + Gx1[133]*Gu1[5] + Gx1[134]*Gu1[9] + Gx1[135]*Gu1[13] + Gx1[136]*Gu1[17] + Gx1[137]*Gu1[21] + Gx1[138]*Gu1[25] + Gx1[139]*Gu1[29] + Gx1[140]*Gu1[33] + Gx1[141]*Gu1[37] + Gx1[142]*Gu1[41] + Gx1[143]*Gu1[45];
Gu2[46] = + Gx1[132]*Gu1[2] + Gx1[133]*Gu1[6] + Gx1[134]*Gu1[10] + Gx1[135]*Gu1[14] + Gx1[136]*Gu1[18] + Gx1[137]*Gu1[22] + Gx1[138]*Gu1[26] + Gx1[139]*Gu1[30] + Gx1[140]*Gu1[34] + Gx1[141]*Gu1[38] + Gx1[142]*Gu1[42] + Gx1[143]*Gu1[46];
Gu2[47] = + Gx1[132]*Gu1[3] + Gx1[133]*Gu1[7] + Gx1[134]*Gu1[11] + Gx1[135]*Gu1[15] + Gx1[136]*Gu1[19] + Gx1[137]*Gu1[23] + Gx1[138]*Gu1[27] + Gx1[139]*Gu1[31] + Gx1[140]*Gu1[35] + Gx1[141]*Gu1[39] + Gx1[142]*Gu1[43] + Gx1[143]*Gu1[47];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
Gu2[32] = Gu1[32];
Gu2[33] = Gu1[33];
Gu2[34] = Gu1[34];
Gu2[35] = Gu1[35];
Gu2[36] = Gu1[36];
Gu2[37] = Gu1[37];
Gu2[38] = Gu1[38];
Gu2[39] = Gu1[39];
Gu2[40] = Gu1[40];
Gu2[41] = Gu1[41];
Gu2[42] = Gu1[42];
Gu2[43] = Gu1[43];
Gu2[44] = Gu1[44];
Gu2[45] = Gu1[45];
Gu2[46] = Gu1[46];
Gu2[47] = Gu1[47];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 192) + (iCol * 4)] += + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36] + Gu1[40]*Gu2[40] + Gu1[44]*Gu2[44];
acadoWorkspace.H[(iRow * 192) + (iCol * 4 + 1)] += + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37] + Gu1[40]*Gu2[41] + Gu1[44]*Gu2[45];
acadoWorkspace.H[(iRow * 192) + (iCol * 4 + 2)] += + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38] + Gu1[40]*Gu2[42] + Gu1[44]*Gu2[46];
acadoWorkspace.H[(iRow * 192) + (iCol * 4 + 3)] += + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39] + Gu1[40]*Gu2[43] + Gu1[44]*Gu2[47];
acadoWorkspace.H[(iRow * 192 + 48) + (iCol * 4)] += + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36] + Gu1[41]*Gu2[40] + Gu1[45]*Gu2[44];
acadoWorkspace.H[(iRow * 192 + 48) + (iCol * 4 + 1)] += + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37] + Gu1[41]*Gu2[41] + Gu1[45]*Gu2[45];
acadoWorkspace.H[(iRow * 192 + 48) + (iCol * 4 + 2)] += + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38] + Gu1[41]*Gu2[42] + Gu1[45]*Gu2[46];
acadoWorkspace.H[(iRow * 192 + 48) + (iCol * 4 + 3)] += + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39] + Gu1[41]*Gu2[43] + Gu1[45]*Gu2[47];
acadoWorkspace.H[(iRow * 192 + 96) + (iCol * 4)] += + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36] + Gu1[42]*Gu2[40] + Gu1[46]*Gu2[44];
acadoWorkspace.H[(iRow * 192 + 96) + (iCol * 4 + 1)] += + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37] + Gu1[42]*Gu2[41] + Gu1[46]*Gu2[45];
acadoWorkspace.H[(iRow * 192 + 96) + (iCol * 4 + 2)] += + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38] + Gu1[42]*Gu2[42] + Gu1[46]*Gu2[46];
acadoWorkspace.H[(iRow * 192 + 96) + (iCol * 4 + 3)] += + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39] + Gu1[42]*Gu2[43] + Gu1[46]*Gu2[47];
acadoWorkspace.H[(iRow * 192 + 144) + (iCol * 4)] += + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36] + Gu1[43]*Gu2[40] + Gu1[47]*Gu2[44];
acadoWorkspace.H[(iRow * 192 + 144) + (iCol * 4 + 1)] += + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37] + Gu1[43]*Gu2[41] + Gu1[47]*Gu2[45];
acadoWorkspace.H[(iRow * 192 + 144) + (iCol * 4 + 2)] += + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38] + Gu1[43]*Gu2[42] + Gu1[47]*Gu2[46];
acadoWorkspace.H[(iRow * 192 + 144) + (iCol * 4 + 3)] += + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39] + Gu1[43]*Gu2[43] + Gu1[47]*Gu2[47];
}

void acado_setBlockH11_R1( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 192) + (iCol * 4)] = (real_t)2.5000000000000000e-01;
acadoWorkspace.H[(iRow * 192) + (iCol * 4 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 192) + (iCol * 4 + 2)] = 0.0;
acadoWorkspace.H[(iRow * 192) + (iCol * 4 + 3)] = 0.0;
acadoWorkspace.H[(iRow * 192 + 48) + (iCol * 4)] = 0.0;
acadoWorkspace.H[(iRow * 192 + 48) + (iCol * 4 + 1)] = (real_t)5.7500000000000000e+00;
acadoWorkspace.H[(iRow * 192 + 48) + (iCol * 4 + 2)] = 0.0;
acadoWorkspace.H[(iRow * 192 + 48) + (iCol * 4 + 3)] = 0.0;
acadoWorkspace.H[(iRow * 192 + 96) + (iCol * 4)] = 0.0;
acadoWorkspace.H[(iRow * 192 + 96) + (iCol * 4 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 192 + 96) + (iCol * 4 + 2)] = (real_t)5.7500000000000000e+00;
acadoWorkspace.H[(iRow * 192 + 96) + (iCol * 4 + 3)] = 0.0;
acadoWorkspace.H[(iRow * 192 + 144) + (iCol * 4)] = 0.0;
acadoWorkspace.H[(iRow * 192 + 144) + (iCol * 4 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 192 + 144) + (iCol * 4 + 2)] = 0.0;
acadoWorkspace.H[(iRow * 192 + 144) + (iCol * 4 + 3)] = (real_t)5.7500000000000000e+00;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 192) + (iCol * 4)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 192) + (iCol * 4 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 192) + (iCol * 4 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 192) + (iCol * 4 + 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 192 + 48) + (iCol * 4)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 192 + 48) + (iCol * 4 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 192 + 48) + (iCol * 4 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 192 + 48) + (iCol * 4 + 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 192 + 96) + (iCol * 4)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 192 + 96) + (iCol * 4 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 192 + 96) + (iCol * 4 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 192 + 96) + (iCol * 4 + 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 192 + 144) + (iCol * 4)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 192 + 144) + (iCol * 4 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 192 + 144) + (iCol * 4 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 192 + 144) + (iCol * 4 + 3)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 192) + (iCol * 4)] = acadoWorkspace.H[(iCol * 192) + (iRow * 4)];
acadoWorkspace.H[(iRow * 192) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 192 + 48) + (iRow * 4)];
acadoWorkspace.H[(iRow * 192) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 192 + 96) + (iRow * 4)];
acadoWorkspace.H[(iRow * 192) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 192 + 144) + (iRow * 4)];
acadoWorkspace.H[(iRow * 192 + 48) + (iCol * 4)] = acadoWorkspace.H[(iCol * 192) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 192 + 48) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 192 + 48) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 192 + 48) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 192 + 96) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 192 + 48) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 192 + 144) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 192 + 96) + (iCol * 4)] = acadoWorkspace.H[(iCol * 192) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 192 + 96) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 192 + 48) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 192 + 96) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 192 + 96) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 192 + 96) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 192 + 144) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 192 + 144) + (iCol * 4)] = acadoWorkspace.H[(iCol * 192) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 192 + 144) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 192 + 48) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 192 + 144) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 192 + 96) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 192 + 144) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 192 + 144) + (iRow * 4 + 3)];
}

void acado_multQ1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = 0.0;
;
dNew[1] = 0.0;
;
dNew[2] = + (real_t)2.5500000000000000e+01*dOld[2];
dNew[3] = + (real_t)1.4750000000000000e+02*dOld[3];
dNew[4] = + (real_t)1.4750000000000000e+02*dOld[4];
dNew[5] = + (real_t)1.4750000000000000e+02*dOld[5];
dNew[6] = 0.0;
;
dNew[7] = 0.0;
;
dNew[8] = 0.0;
;
dNew[9] = 0.0;
;
dNew[10] = 0.0;
;
dNew[11] = 0.0;
;
}

void acado_multQN1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = 0.0;
;
dNew[1] = 0.0;
;
dNew[2] = + (real_t)2.5000000000000001e-02*dOld[2];
dNew[3] = + (real_t)2.5000000000000000e-01*dOld[3];
dNew[4] = + (real_t)2.5000000000000000e-01*dOld[4];
dNew[5] = + (real_t)2.5000000000000000e-01*dOld[5];
dNew[6] = 0.0;
;
dNew[7] = 0.0;
;
dNew[8] = 0.0;
;
dNew[9] = 0.0;
;
dNew[10] = 0.0;
;
dNew[11] = 0.0;
;
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + (real_t)2.5000000000000000e-01*Dy1[4];
RDy1[1] = + (real_t)5.7500000000000000e+00*Dy1[5];
RDy1[2] = + (real_t)5.7500000000000000e+00*Dy1[6];
RDy1[3] = + (real_t)5.7500000000000000e+00*Dy1[7];
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = 0.0;
;
QDy1[1] = 0.0;
;
QDy1[2] = + (real_t)2.5500000000000000e+01*Dy1[0];
QDy1[3] = + (real_t)1.4750000000000000e+02*Dy1[1];
QDy1[4] = + (real_t)1.4750000000000000e+02*Dy1[2];
QDy1[5] = + (real_t)1.4750000000000000e+02*Dy1[3];
QDy1[6] = 0.0;
;
QDy1[7] = 0.0;
;
QDy1[8] = 0.0;
;
QDy1[9] = 0.0;
;
QDy1[10] = 0.0;
;
QDy1[11] = 0.0;
;
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[4]*QDy1[1] + E1[8]*QDy1[2] + E1[12]*QDy1[3] + E1[16]*QDy1[4] + E1[20]*QDy1[5] + E1[24]*QDy1[6] + E1[28]*QDy1[7] + E1[32]*QDy1[8] + E1[36]*QDy1[9] + E1[40]*QDy1[10] + E1[44]*QDy1[11];
U1[1] += + E1[1]*QDy1[0] + E1[5]*QDy1[1] + E1[9]*QDy1[2] + E1[13]*QDy1[3] + E1[17]*QDy1[4] + E1[21]*QDy1[5] + E1[25]*QDy1[6] + E1[29]*QDy1[7] + E1[33]*QDy1[8] + E1[37]*QDy1[9] + E1[41]*QDy1[10] + E1[45]*QDy1[11];
U1[2] += + E1[2]*QDy1[0] + E1[6]*QDy1[1] + E1[10]*QDy1[2] + E1[14]*QDy1[3] + E1[18]*QDy1[4] + E1[22]*QDy1[5] + E1[26]*QDy1[6] + E1[30]*QDy1[7] + E1[34]*QDy1[8] + E1[38]*QDy1[9] + E1[42]*QDy1[10] + E1[46]*QDy1[11];
U1[3] += + E1[3]*QDy1[0] + E1[7]*QDy1[1] + E1[11]*QDy1[2] + E1[15]*QDy1[3] + E1[19]*QDy1[4] + E1[23]*QDy1[5] + E1[27]*QDy1[6] + E1[31]*QDy1[7] + E1[35]*QDy1[8] + E1[39]*QDy1[9] + E1[43]*QDy1[10] + E1[47]*QDy1[11];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[4]*Gx1[12] + E1[8]*Gx1[24] + E1[12]*Gx1[36] + E1[16]*Gx1[48] + E1[20]*Gx1[60] + E1[24]*Gx1[72] + E1[28]*Gx1[84] + E1[32]*Gx1[96] + E1[36]*Gx1[108] + E1[40]*Gx1[120] + E1[44]*Gx1[132];
H101[1] += + E1[0]*Gx1[1] + E1[4]*Gx1[13] + E1[8]*Gx1[25] + E1[12]*Gx1[37] + E1[16]*Gx1[49] + E1[20]*Gx1[61] + E1[24]*Gx1[73] + E1[28]*Gx1[85] + E1[32]*Gx1[97] + E1[36]*Gx1[109] + E1[40]*Gx1[121] + E1[44]*Gx1[133];
H101[2] += + E1[0]*Gx1[2] + E1[4]*Gx1[14] + E1[8]*Gx1[26] + E1[12]*Gx1[38] + E1[16]*Gx1[50] + E1[20]*Gx1[62] + E1[24]*Gx1[74] + E1[28]*Gx1[86] + E1[32]*Gx1[98] + E1[36]*Gx1[110] + E1[40]*Gx1[122] + E1[44]*Gx1[134];
H101[3] += + E1[0]*Gx1[3] + E1[4]*Gx1[15] + E1[8]*Gx1[27] + E1[12]*Gx1[39] + E1[16]*Gx1[51] + E1[20]*Gx1[63] + E1[24]*Gx1[75] + E1[28]*Gx1[87] + E1[32]*Gx1[99] + E1[36]*Gx1[111] + E1[40]*Gx1[123] + E1[44]*Gx1[135];
H101[4] += + E1[0]*Gx1[4] + E1[4]*Gx1[16] + E1[8]*Gx1[28] + E1[12]*Gx1[40] + E1[16]*Gx1[52] + E1[20]*Gx1[64] + E1[24]*Gx1[76] + E1[28]*Gx1[88] + E1[32]*Gx1[100] + E1[36]*Gx1[112] + E1[40]*Gx1[124] + E1[44]*Gx1[136];
H101[5] += + E1[0]*Gx1[5] + E1[4]*Gx1[17] + E1[8]*Gx1[29] + E1[12]*Gx1[41] + E1[16]*Gx1[53] + E1[20]*Gx1[65] + E1[24]*Gx1[77] + E1[28]*Gx1[89] + E1[32]*Gx1[101] + E1[36]*Gx1[113] + E1[40]*Gx1[125] + E1[44]*Gx1[137];
H101[6] += + E1[0]*Gx1[6] + E1[4]*Gx1[18] + E1[8]*Gx1[30] + E1[12]*Gx1[42] + E1[16]*Gx1[54] + E1[20]*Gx1[66] + E1[24]*Gx1[78] + E1[28]*Gx1[90] + E1[32]*Gx1[102] + E1[36]*Gx1[114] + E1[40]*Gx1[126] + E1[44]*Gx1[138];
H101[7] += + E1[0]*Gx1[7] + E1[4]*Gx1[19] + E1[8]*Gx1[31] + E1[12]*Gx1[43] + E1[16]*Gx1[55] + E1[20]*Gx1[67] + E1[24]*Gx1[79] + E1[28]*Gx1[91] + E1[32]*Gx1[103] + E1[36]*Gx1[115] + E1[40]*Gx1[127] + E1[44]*Gx1[139];
H101[8] += + E1[0]*Gx1[8] + E1[4]*Gx1[20] + E1[8]*Gx1[32] + E1[12]*Gx1[44] + E1[16]*Gx1[56] + E1[20]*Gx1[68] + E1[24]*Gx1[80] + E1[28]*Gx1[92] + E1[32]*Gx1[104] + E1[36]*Gx1[116] + E1[40]*Gx1[128] + E1[44]*Gx1[140];
H101[9] += + E1[0]*Gx1[9] + E1[4]*Gx1[21] + E1[8]*Gx1[33] + E1[12]*Gx1[45] + E1[16]*Gx1[57] + E1[20]*Gx1[69] + E1[24]*Gx1[81] + E1[28]*Gx1[93] + E1[32]*Gx1[105] + E1[36]*Gx1[117] + E1[40]*Gx1[129] + E1[44]*Gx1[141];
H101[10] += + E1[0]*Gx1[10] + E1[4]*Gx1[22] + E1[8]*Gx1[34] + E1[12]*Gx1[46] + E1[16]*Gx1[58] + E1[20]*Gx1[70] + E1[24]*Gx1[82] + E1[28]*Gx1[94] + E1[32]*Gx1[106] + E1[36]*Gx1[118] + E1[40]*Gx1[130] + E1[44]*Gx1[142];
H101[11] += + E1[0]*Gx1[11] + E1[4]*Gx1[23] + E1[8]*Gx1[35] + E1[12]*Gx1[47] + E1[16]*Gx1[59] + E1[20]*Gx1[71] + E1[24]*Gx1[83] + E1[28]*Gx1[95] + E1[32]*Gx1[107] + E1[36]*Gx1[119] + E1[40]*Gx1[131] + E1[44]*Gx1[143];
H101[12] += + E1[1]*Gx1[0] + E1[5]*Gx1[12] + E1[9]*Gx1[24] + E1[13]*Gx1[36] + E1[17]*Gx1[48] + E1[21]*Gx1[60] + E1[25]*Gx1[72] + E1[29]*Gx1[84] + E1[33]*Gx1[96] + E1[37]*Gx1[108] + E1[41]*Gx1[120] + E1[45]*Gx1[132];
H101[13] += + E1[1]*Gx1[1] + E1[5]*Gx1[13] + E1[9]*Gx1[25] + E1[13]*Gx1[37] + E1[17]*Gx1[49] + E1[21]*Gx1[61] + E1[25]*Gx1[73] + E1[29]*Gx1[85] + E1[33]*Gx1[97] + E1[37]*Gx1[109] + E1[41]*Gx1[121] + E1[45]*Gx1[133];
H101[14] += + E1[1]*Gx1[2] + E1[5]*Gx1[14] + E1[9]*Gx1[26] + E1[13]*Gx1[38] + E1[17]*Gx1[50] + E1[21]*Gx1[62] + E1[25]*Gx1[74] + E1[29]*Gx1[86] + E1[33]*Gx1[98] + E1[37]*Gx1[110] + E1[41]*Gx1[122] + E1[45]*Gx1[134];
H101[15] += + E1[1]*Gx1[3] + E1[5]*Gx1[15] + E1[9]*Gx1[27] + E1[13]*Gx1[39] + E1[17]*Gx1[51] + E1[21]*Gx1[63] + E1[25]*Gx1[75] + E1[29]*Gx1[87] + E1[33]*Gx1[99] + E1[37]*Gx1[111] + E1[41]*Gx1[123] + E1[45]*Gx1[135];
H101[16] += + E1[1]*Gx1[4] + E1[5]*Gx1[16] + E1[9]*Gx1[28] + E1[13]*Gx1[40] + E1[17]*Gx1[52] + E1[21]*Gx1[64] + E1[25]*Gx1[76] + E1[29]*Gx1[88] + E1[33]*Gx1[100] + E1[37]*Gx1[112] + E1[41]*Gx1[124] + E1[45]*Gx1[136];
H101[17] += + E1[1]*Gx1[5] + E1[5]*Gx1[17] + E1[9]*Gx1[29] + E1[13]*Gx1[41] + E1[17]*Gx1[53] + E1[21]*Gx1[65] + E1[25]*Gx1[77] + E1[29]*Gx1[89] + E1[33]*Gx1[101] + E1[37]*Gx1[113] + E1[41]*Gx1[125] + E1[45]*Gx1[137];
H101[18] += + E1[1]*Gx1[6] + E1[5]*Gx1[18] + E1[9]*Gx1[30] + E1[13]*Gx1[42] + E1[17]*Gx1[54] + E1[21]*Gx1[66] + E1[25]*Gx1[78] + E1[29]*Gx1[90] + E1[33]*Gx1[102] + E1[37]*Gx1[114] + E1[41]*Gx1[126] + E1[45]*Gx1[138];
H101[19] += + E1[1]*Gx1[7] + E1[5]*Gx1[19] + E1[9]*Gx1[31] + E1[13]*Gx1[43] + E1[17]*Gx1[55] + E1[21]*Gx1[67] + E1[25]*Gx1[79] + E1[29]*Gx1[91] + E1[33]*Gx1[103] + E1[37]*Gx1[115] + E1[41]*Gx1[127] + E1[45]*Gx1[139];
H101[20] += + E1[1]*Gx1[8] + E1[5]*Gx1[20] + E1[9]*Gx1[32] + E1[13]*Gx1[44] + E1[17]*Gx1[56] + E1[21]*Gx1[68] + E1[25]*Gx1[80] + E1[29]*Gx1[92] + E1[33]*Gx1[104] + E1[37]*Gx1[116] + E1[41]*Gx1[128] + E1[45]*Gx1[140];
H101[21] += + E1[1]*Gx1[9] + E1[5]*Gx1[21] + E1[9]*Gx1[33] + E1[13]*Gx1[45] + E1[17]*Gx1[57] + E1[21]*Gx1[69] + E1[25]*Gx1[81] + E1[29]*Gx1[93] + E1[33]*Gx1[105] + E1[37]*Gx1[117] + E1[41]*Gx1[129] + E1[45]*Gx1[141];
H101[22] += + E1[1]*Gx1[10] + E1[5]*Gx1[22] + E1[9]*Gx1[34] + E1[13]*Gx1[46] + E1[17]*Gx1[58] + E1[21]*Gx1[70] + E1[25]*Gx1[82] + E1[29]*Gx1[94] + E1[33]*Gx1[106] + E1[37]*Gx1[118] + E1[41]*Gx1[130] + E1[45]*Gx1[142];
H101[23] += + E1[1]*Gx1[11] + E1[5]*Gx1[23] + E1[9]*Gx1[35] + E1[13]*Gx1[47] + E1[17]*Gx1[59] + E1[21]*Gx1[71] + E1[25]*Gx1[83] + E1[29]*Gx1[95] + E1[33]*Gx1[107] + E1[37]*Gx1[119] + E1[41]*Gx1[131] + E1[45]*Gx1[143];
H101[24] += + E1[2]*Gx1[0] + E1[6]*Gx1[12] + E1[10]*Gx1[24] + E1[14]*Gx1[36] + E1[18]*Gx1[48] + E1[22]*Gx1[60] + E1[26]*Gx1[72] + E1[30]*Gx1[84] + E1[34]*Gx1[96] + E1[38]*Gx1[108] + E1[42]*Gx1[120] + E1[46]*Gx1[132];
H101[25] += + E1[2]*Gx1[1] + E1[6]*Gx1[13] + E1[10]*Gx1[25] + E1[14]*Gx1[37] + E1[18]*Gx1[49] + E1[22]*Gx1[61] + E1[26]*Gx1[73] + E1[30]*Gx1[85] + E1[34]*Gx1[97] + E1[38]*Gx1[109] + E1[42]*Gx1[121] + E1[46]*Gx1[133];
H101[26] += + E1[2]*Gx1[2] + E1[6]*Gx1[14] + E1[10]*Gx1[26] + E1[14]*Gx1[38] + E1[18]*Gx1[50] + E1[22]*Gx1[62] + E1[26]*Gx1[74] + E1[30]*Gx1[86] + E1[34]*Gx1[98] + E1[38]*Gx1[110] + E1[42]*Gx1[122] + E1[46]*Gx1[134];
H101[27] += + E1[2]*Gx1[3] + E1[6]*Gx1[15] + E1[10]*Gx1[27] + E1[14]*Gx1[39] + E1[18]*Gx1[51] + E1[22]*Gx1[63] + E1[26]*Gx1[75] + E1[30]*Gx1[87] + E1[34]*Gx1[99] + E1[38]*Gx1[111] + E1[42]*Gx1[123] + E1[46]*Gx1[135];
H101[28] += + E1[2]*Gx1[4] + E1[6]*Gx1[16] + E1[10]*Gx1[28] + E1[14]*Gx1[40] + E1[18]*Gx1[52] + E1[22]*Gx1[64] + E1[26]*Gx1[76] + E1[30]*Gx1[88] + E1[34]*Gx1[100] + E1[38]*Gx1[112] + E1[42]*Gx1[124] + E1[46]*Gx1[136];
H101[29] += + E1[2]*Gx1[5] + E1[6]*Gx1[17] + E1[10]*Gx1[29] + E1[14]*Gx1[41] + E1[18]*Gx1[53] + E1[22]*Gx1[65] + E1[26]*Gx1[77] + E1[30]*Gx1[89] + E1[34]*Gx1[101] + E1[38]*Gx1[113] + E1[42]*Gx1[125] + E1[46]*Gx1[137];
H101[30] += + E1[2]*Gx1[6] + E1[6]*Gx1[18] + E1[10]*Gx1[30] + E1[14]*Gx1[42] + E1[18]*Gx1[54] + E1[22]*Gx1[66] + E1[26]*Gx1[78] + E1[30]*Gx1[90] + E1[34]*Gx1[102] + E1[38]*Gx1[114] + E1[42]*Gx1[126] + E1[46]*Gx1[138];
H101[31] += + E1[2]*Gx1[7] + E1[6]*Gx1[19] + E1[10]*Gx1[31] + E1[14]*Gx1[43] + E1[18]*Gx1[55] + E1[22]*Gx1[67] + E1[26]*Gx1[79] + E1[30]*Gx1[91] + E1[34]*Gx1[103] + E1[38]*Gx1[115] + E1[42]*Gx1[127] + E1[46]*Gx1[139];
H101[32] += + E1[2]*Gx1[8] + E1[6]*Gx1[20] + E1[10]*Gx1[32] + E1[14]*Gx1[44] + E1[18]*Gx1[56] + E1[22]*Gx1[68] + E1[26]*Gx1[80] + E1[30]*Gx1[92] + E1[34]*Gx1[104] + E1[38]*Gx1[116] + E1[42]*Gx1[128] + E1[46]*Gx1[140];
H101[33] += + E1[2]*Gx1[9] + E1[6]*Gx1[21] + E1[10]*Gx1[33] + E1[14]*Gx1[45] + E1[18]*Gx1[57] + E1[22]*Gx1[69] + E1[26]*Gx1[81] + E1[30]*Gx1[93] + E1[34]*Gx1[105] + E1[38]*Gx1[117] + E1[42]*Gx1[129] + E1[46]*Gx1[141];
H101[34] += + E1[2]*Gx1[10] + E1[6]*Gx1[22] + E1[10]*Gx1[34] + E1[14]*Gx1[46] + E1[18]*Gx1[58] + E1[22]*Gx1[70] + E1[26]*Gx1[82] + E1[30]*Gx1[94] + E1[34]*Gx1[106] + E1[38]*Gx1[118] + E1[42]*Gx1[130] + E1[46]*Gx1[142];
H101[35] += + E1[2]*Gx1[11] + E1[6]*Gx1[23] + E1[10]*Gx1[35] + E1[14]*Gx1[47] + E1[18]*Gx1[59] + E1[22]*Gx1[71] + E1[26]*Gx1[83] + E1[30]*Gx1[95] + E1[34]*Gx1[107] + E1[38]*Gx1[119] + E1[42]*Gx1[131] + E1[46]*Gx1[143];
H101[36] += + E1[3]*Gx1[0] + E1[7]*Gx1[12] + E1[11]*Gx1[24] + E1[15]*Gx1[36] + E1[19]*Gx1[48] + E1[23]*Gx1[60] + E1[27]*Gx1[72] + E1[31]*Gx1[84] + E1[35]*Gx1[96] + E1[39]*Gx1[108] + E1[43]*Gx1[120] + E1[47]*Gx1[132];
H101[37] += + E1[3]*Gx1[1] + E1[7]*Gx1[13] + E1[11]*Gx1[25] + E1[15]*Gx1[37] + E1[19]*Gx1[49] + E1[23]*Gx1[61] + E1[27]*Gx1[73] + E1[31]*Gx1[85] + E1[35]*Gx1[97] + E1[39]*Gx1[109] + E1[43]*Gx1[121] + E1[47]*Gx1[133];
H101[38] += + E1[3]*Gx1[2] + E1[7]*Gx1[14] + E1[11]*Gx1[26] + E1[15]*Gx1[38] + E1[19]*Gx1[50] + E1[23]*Gx1[62] + E1[27]*Gx1[74] + E1[31]*Gx1[86] + E1[35]*Gx1[98] + E1[39]*Gx1[110] + E1[43]*Gx1[122] + E1[47]*Gx1[134];
H101[39] += + E1[3]*Gx1[3] + E1[7]*Gx1[15] + E1[11]*Gx1[27] + E1[15]*Gx1[39] + E1[19]*Gx1[51] + E1[23]*Gx1[63] + E1[27]*Gx1[75] + E1[31]*Gx1[87] + E1[35]*Gx1[99] + E1[39]*Gx1[111] + E1[43]*Gx1[123] + E1[47]*Gx1[135];
H101[40] += + E1[3]*Gx1[4] + E1[7]*Gx1[16] + E1[11]*Gx1[28] + E1[15]*Gx1[40] + E1[19]*Gx1[52] + E1[23]*Gx1[64] + E1[27]*Gx1[76] + E1[31]*Gx1[88] + E1[35]*Gx1[100] + E1[39]*Gx1[112] + E1[43]*Gx1[124] + E1[47]*Gx1[136];
H101[41] += + E1[3]*Gx1[5] + E1[7]*Gx1[17] + E1[11]*Gx1[29] + E1[15]*Gx1[41] + E1[19]*Gx1[53] + E1[23]*Gx1[65] + E1[27]*Gx1[77] + E1[31]*Gx1[89] + E1[35]*Gx1[101] + E1[39]*Gx1[113] + E1[43]*Gx1[125] + E1[47]*Gx1[137];
H101[42] += + E1[3]*Gx1[6] + E1[7]*Gx1[18] + E1[11]*Gx1[30] + E1[15]*Gx1[42] + E1[19]*Gx1[54] + E1[23]*Gx1[66] + E1[27]*Gx1[78] + E1[31]*Gx1[90] + E1[35]*Gx1[102] + E1[39]*Gx1[114] + E1[43]*Gx1[126] + E1[47]*Gx1[138];
H101[43] += + E1[3]*Gx1[7] + E1[7]*Gx1[19] + E1[11]*Gx1[31] + E1[15]*Gx1[43] + E1[19]*Gx1[55] + E1[23]*Gx1[67] + E1[27]*Gx1[79] + E1[31]*Gx1[91] + E1[35]*Gx1[103] + E1[39]*Gx1[115] + E1[43]*Gx1[127] + E1[47]*Gx1[139];
H101[44] += + E1[3]*Gx1[8] + E1[7]*Gx1[20] + E1[11]*Gx1[32] + E1[15]*Gx1[44] + E1[19]*Gx1[56] + E1[23]*Gx1[68] + E1[27]*Gx1[80] + E1[31]*Gx1[92] + E1[35]*Gx1[104] + E1[39]*Gx1[116] + E1[43]*Gx1[128] + E1[47]*Gx1[140];
H101[45] += + E1[3]*Gx1[9] + E1[7]*Gx1[21] + E1[11]*Gx1[33] + E1[15]*Gx1[45] + E1[19]*Gx1[57] + E1[23]*Gx1[69] + E1[27]*Gx1[81] + E1[31]*Gx1[93] + E1[35]*Gx1[105] + E1[39]*Gx1[117] + E1[43]*Gx1[129] + E1[47]*Gx1[141];
H101[46] += + E1[3]*Gx1[10] + E1[7]*Gx1[22] + E1[11]*Gx1[34] + E1[15]*Gx1[46] + E1[19]*Gx1[58] + E1[23]*Gx1[70] + E1[27]*Gx1[82] + E1[31]*Gx1[94] + E1[35]*Gx1[106] + E1[39]*Gx1[118] + E1[43]*Gx1[130] + E1[47]*Gx1[142];
H101[47] += + E1[3]*Gx1[11] + E1[7]*Gx1[23] + E1[11]*Gx1[35] + E1[15]*Gx1[47] + E1[19]*Gx1[59] + E1[23]*Gx1[71] + E1[27]*Gx1[83] + E1[31]*Gx1[95] + E1[35]*Gx1[107] + E1[39]*Gx1[119] + E1[43]*Gx1[131] + E1[47]*Gx1[143];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 48; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1] + E1[2]*U1[2] + E1[3]*U1[3];
dNew[1] += + E1[4]*U1[0] + E1[5]*U1[1] + E1[6]*U1[2] + E1[7]*U1[3];
dNew[2] += + E1[8]*U1[0] + E1[9]*U1[1] + E1[10]*U1[2] + E1[11]*U1[3];
dNew[3] += + E1[12]*U1[0] + E1[13]*U1[1] + E1[14]*U1[2] + E1[15]*U1[3];
dNew[4] += + E1[16]*U1[0] + E1[17]*U1[1] + E1[18]*U1[2] + E1[19]*U1[3];
dNew[5] += + E1[20]*U1[0] + E1[21]*U1[1] + E1[22]*U1[2] + E1[23]*U1[3];
dNew[6] += + E1[24]*U1[0] + E1[25]*U1[1] + E1[26]*U1[2] + E1[27]*U1[3];
dNew[7] += + E1[28]*U1[0] + E1[29]*U1[1] + E1[30]*U1[2] + E1[31]*U1[3];
dNew[8] += + E1[32]*U1[0] + E1[33]*U1[1] + E1[34]*U1[2] + E1[35]*U1[3];
dNew[9] += + E1[36]*U1[0] + E1[37]*U1[1] + E1[38]*U1[2] + E1[39]*U1[3];
dNew[10] += + E1[40]*U1[0] + E1[41]*U1[1] + E1[42]*U1[2] + E1[43]*U1[3];
dNew[11] += + E1[44]*U1[0] + E1[45]*U1[1] + E1[46]*U1[2] + E1[47]*U1[3];
}

void acado_multQ1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = 0.0;
;
Gx2[1] = 0.0;
;
Gx2[2] = 0.0;
;
Gx2[3] = 0.0;
;
Gx2[4] = 0.0;
;
Gx2[5] = 0.0;
;
Gx2[6] = 0.0;
;
Gx2[7] = 0.0;
;
Gx2[8] = 0.0;
;
Gx2[9] = 0.0;
;
Gx2[10] = 0.0;
;
Gx2[11] = 0.0;
;
Gx2[12] = 0.0;
;
Gx2[13] = 0.0;
;
Gx2[14] = 0.0;
;
Gx2[15] = 0.0;
;
Gx2[16] = 0.0;
;
Gx2[17] = 0.0;
;
Gx2[18] = 0.0;
;
Gx2[19] = 0.0;
;
Gx2[20] = 0.0;
;
Gx2[21] = 0.0;
;
Gx2[22] = 0.0;
;
Gx2[23] = 0.0;
;
Gx2[24] = + (real_t)2.5500000000000000e+01*Gx1[24];
Gx2[25] = + (real_t)2.5500000000000000e+01*Gx1[25];
Gx2[26] = + (real_t)2.5500000000000000e+01*Gx1[26];
Gx2[27] = + (real_t)2.5500000000000000e+01*Gx1[27];
Gx2[28] = + (real_t)2.5500000000000000e+01*Gx1[28];
Gx2[29] = + (real_t)2.5500000000000000e+01*Gx1[29];
Gx2[30] = + (real_t)2.5500000000000000e+01*Gx1[30];
Gx2[31] = + (real_t)2.5500000000000000e+01*Gx1[31];
Gx2[32] = + (real_t)2.5500000000000000e+01*Gx1[32];
Gx2[33] = + (real_t)2.5500000000000000e+01*Gx1[33];
Gx2[34] = + (real_t)2.5500000000000000e+01*Gx1[34];
Gx2[35] = + (real_t)2.5500000000000000e+01*Gx1[35];
Gx2[36] = + (real_t)1.4750000000000000e+02*Gx1[36];
Gx2[37] = + (real_t)1.4750000000000000e+02*Gx1[37];
Gx2[38] = + (real_t)1.4750000000000000e+02*Gx1[38];
Gx2[39] = + (real_t)1.4750000000000000e+02*Gx1[39];
Gx2[40] = + (real_t)1.4750000000000000e+02*Gx1[40];
Gx2[41] = + (real_t)1.4750000000000000e+02*Gx1[41];
Gx2[42] = + (real_t)1.4750000000000000e+02*Gx1[42];
Gx2[43] = + (real_t)1.4750000000000000e+02*Gx1[43];
Gx2[44] = + (real_t)1.4750000000000000e+02*Gx1[44];
Gx2[45] = + (real_t)1.4750000000000000e+02*Gx1[45];
Gx2[46] = + (real_t)1.4750000000000000e+02*Gx1[46];
Gx2[47] = + (real_t)1.4750000000000000e+02*Gx1[47];
Gx2[48] = + (real_t)1.4750000000000000e+02*Gx1[48];
Gx2[49] = + (real_t)1.4750000000000000e+02*Gx1[49];
Gx2[50] = + (real_t)1.4750000000000000e+02*Gx1[50];
Gx2[51] = + (real_t)1.4750000000000000e+02*Gx1[51];
Gx2[52] = + (real_t)1.4750000000000000e+02*Gx1[52];
Gx2[53] = + (real_t)1.4750000000000000e+02*Gx1[53];
Gx2[54] = + (real_t)1.4750000000000000e+02*Gx1[54];
Gx2[55] = + (real_t)1.4750000000000000e+02*Gx1[55];
Gx2[56] = + (real_t)1.4750000000000000e+02*Gx1[56];
Gx2[57] = + (real_t)1.4750000000000000e+02*Gx1[57];
Gx2[58] = + (real_t)1.4750000000000000e+02*Gx1[58];
Gx2[59] = + (real_t)1.4750000000000000e+02*Gx1[59];
Gx2[60] = + (real_t)1.4750000000000000e+02*Gx1[60];
Gx2[61] = + (real_t)1.4750000000000000e+02*Gx1[61];
Gx2[62] = + (real_t)1.4750000000000000e+02*Gx1[62];
Gx2[63] = + (real_t)1.4750000000000000e+02*Gx1[63];
Gx2[64] = + (real_t)1.4750000000000000e+02*Gx1[64];
Gx2[65] = + (real_t)1.4750000000000000e+02*Gx1[65];
Gx2[66] = + (real_t)1.4750000000000000e+02*Gx1[66];
Gx2[67] = + (real_t)1.4750000000000000e+02*Gx1[67];
Gx2[68] = + (real_t)1.4750000000000000e+02*Gx1[68];
Gx2[69] = + (real_t)1.4750000000000000e+02*Gx1[69];
Gx2[70] = + (real_t)1.4750000000000000e+02*Gx1[70];
Gx2[71] = + (real_t)1.4750000000000000e+02*Gx1[71];
Gx2[72] = 0.0;
;
Gx2[73] = 0.0;
;
Gx2[74] = 0.0;
;
Gx2[75] = 0.0;
;
Gx2[76] = 0.0;
;
Gx2[77] = 0.0;
;
Gx2[78] = 0.0;
;
Gx2[79] = 0.0;
;
Gx2[80] = 0.0;
;
Gx2[81] = 0.0;
;
Gx2[82] = 0.0;
;
Gx2[83] = 0.0;
;
Gx2[84] = 0.0;
;
Gx2[85] = 0.0;
;
Gx2[86] = 0.0;
;
Gx2[87] = 0.0;
;
Gx2[88] = 0.0;
;
Gx2[89] = 0.0;
;
Gx2[90] = 0.0;
;
Gx2[91] = 0.0;
;
Gx2[92] = 0.0;
;
Gx2[93] = 0.0;
;
Gx2[94] = 0.0;
;
Gx2[95] = 0.0;
;
Gx2[96] = 0.0;
;
Gx2[97] = 0.0;
;
Gx2[98] = 0.0;
;
Gx2[99] = 0.0;
;
Gx2[100] = 0.0;
;
Gx2[101] = 0.0;
;
Gx2[102] = 0.0;
;
Gx2[103] = 0.0;
;
Gx2[104] = 0.0;
;
Gx2[105] = 0.0;
;
Gx2[106] = 0.0;
;
Gx2[107] = 0.0;
;
Gx2[108] = 0.0;
;
Gx2[109] = 0.0;
;
Gx2[110] = 0.0;
;
Gx2[111] = 0.0;
;
Gx2[112] = 0.0;
;
Gx2[113] = 0.0;
;
Gx2[114] = 0.0;
;
Gx2[115] = 0.0;
;
Gx2[116] = 0.0;
;
Gx2[117] = 0.0;
;
Gx2[118] = 0.0;
;
Gx2[119] = 0.0;
;
Gx2[120] = 0.0;
;
Gx2[121] = 0.0;
;
Gx2[122] = 0.0;
;
Gx2[123] = 0.0;
;
Gx2[124] = 0.0;
;
Gx2[125] = 0.0;
;
Gx2[126] = 0.0;
;
Gx2[127] = 0.0;
;
Gx2[128] = 0.0;
;
Gx2[129] = 0.0;
;
Gx2[130] = 0.0;
;
Gx2[131] = 0.0;
;
Gx2[132] = 0.0;
;
Gx2[133] = 0.0;
;
Gx2[134] = 0.0;
;
Gx2[135] = 0.0;
;
Gx2[136] = 0.0;
;
Gx2[137] = 0.0;
;
Gx2[138] = 0.0;
;
Gx2[139] = 0.0;
;
Gx2[140] = 0.0;
;
Gx2[141] = 0.0;
;
Gx2[142] = 0.0;
;
Gx2[143] = 0.0;
;
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = 0.0;
;
Gx2[1] = 0.0;
;
Gx2[2] = 0.0;
;
Gx2[3] = 0.0;
;
Gx2[4] = 0.0;
;
Gx2[5] = 0.0;
;
Gx2[6] = 0.0;
;
Gx2[7] = 0.0;
;
Gx2[8] = 0.0;
;
Gx2[9] = 0.0;
;
Gx2[10] = 0.0;
;
Gx2[11] = 0.0;
;
Gx2[12] = 0.0;
;
Gx2[13] = 0.0;
;
Gx2[14] = 0.0;
;
Gx2[15] = 0.0;
;
Gx2[16] = 0.0;
;
Gx2[17] = 0.0;
;
Gx2[18] = 0.0;
;
Gx2[19] = 0.0;
;
Gx2[20] = 0.0;
;
Gx2[21] = 0.0;
;
Gx2[22] = 0.0;
;
Gx2[23] = 0.0;
;
Gx2[24] = + (real_t)2.5000000000000001e-02*Gx1[24];
Gx2[25] = + (real_t)2.5000000000000001e-02*Gx1[25];
Gx2[26] = + (real_t)2.5000000000000001e-02*Gx1[26];
Gx2[27] = + (real_t)2.5000000000000001e-02*Gx1[27];
Gx2[28] = + (real_t)2.5000000000000001e-02*Gx1[28];
Gx2[29] = + (real_t)2.5000000000000001e-02*Gx1[29];
Gx2[30] = + (real_t)2.5000000000000001e-02*Gx1[30];
Gx2[31] = + (real_t)2.5000000000000001e-02*Gx1[31];
Gx2[32] = + (real_t)2.5000000000000001e-02*Gx1[32];
Gx2[33] = + (real_t)2.5000000000000001e-02*Gx1[33];
Gx2[34] = + (real_t)2.5000000000000001e-02*Gx1[34];
Gx2[35] = + (real_t)2.5000000000000001e-02*Gx1[35];
Gx2[36] = + (real_t)2.5000000000000000e-01*Gx1[36];
Gx2[37] = + (real_t)2.5000000000000000e-01*Gx1[37];
Gx2[38] = + (real_t)2.5000000000000000e-01*Gx1[38];
Gx2[39] = + (real_t)2.5000000000000000e-01*Gx1[39];
Gx2[40] = + (real_t)2.5000000000000000e-01*Gx1[40];
Gx2[41] = + (real_t)2.5000000000000000e-01*Gx1[41];
Gx2[42] = + (real_t)2.5000000000000000e-01*Gx1[42];
Gx2[43] = + (real_t)2.5000000000000000e-01*Gx1[43];
Gx2[44] = + (real_t)2.5000000000000000e-01*Gx1[44];
Gx2[45] = + (real_t)2.5000000000000000e-01*Gx1[45];
Gx2[46] = + (real_t)2.5000000000000000e-01*Gx1[46];
Gx2[47] = + (real_t)2.5000000000000000e-01*Gx1[47];
Gx2[48] = + (real_t)2.5000000000000000e-01*Gx1[48];
Gx2[49] = + (real_t)2.5000000000000000e-01*Gx1[49];
Gx2[50] = + (real_t)2.5000000000000000e-01*Gx1[50];
Gx2[51] = + (real_t)2.5000000000000000e-01*Gx1[51];
Gx2[52] = + (real_t)2.5000000000000000e-01*Gx1[52];
Gx2[53] = + (real_t)2.5000000000000000e-01*Gx1[53];
Gx2[54] = + (real_t)2.5000000000000000e-01*Gx1[54];
Gx2[55] = + (real_t)2.5000000000000000e-01*Gx1[55];
Gx2[56] = + (real_t)2.5000000000000000e-01*Gx1[56];
Gx2[57] = + (real_t)2.5000000000000000e-01*Gx1[57];
Gx2[58] = + (real_t)2.5000000000000000e-01*Gx1[58];
Gx2[59] = + (real_t)2.5000000000000000e-01*Gx1[59];
Gx2[60] = + (real_t)2.5000000000000000e-01*Gx1[60];
Gx2[61] = + (real_t)2.5000000000000000e-01*Gx1[61];
Gx2[62] = + (real_t)2.5000000000000000e-01*Gx1[62];
Gx2[63] = + (real_t)2.5000000000000000e-01*Gx1[63];
Gx2[64] = + (real_t)2.5000000000000000e-01*Gx1[64];
Gx2[65] = + (real_t)2.5000000000000000e-01*Gx1[65];
Gx2[66] = + (real_t)2.5000000000000000e-01*Gx1[66];
Gx2[67] = + (real_t)2.5000000000000000e-01*Gx1[67];
Gx2[68] = + (real_t)2.5000000000000000e-01*Gx1[68];
Gx2[69] = + (real_t)2.5000000000000000e-01*Gx1[69];
Gx2[70] = + (real_t)2.5000000000000000e-01*Gx1[70];
Gx2[71] = + (real_t)2.5000000000000000e-01*Gx1[71];
Gx2[72] = 0.0;
;
Gx2[73] = 0.0;
;
Gx2[74] = 0.0;
;
Gx2[75] = 0.0;
;
Gx2[76] = 0.0;
;
Gx2[77] = 0.0;
;
Gx2[78] = 0.0;
;
Gx2[79] = 0.0;
;
Gx2[80] = 0.0;
;
Gx2[81] = 0.0;
;
Gx2[82] = 0.0;
;
Gx2[83] = 0.0;
;
Gx2[84] = 0.0;
;
Gx2[85] = 0.0;
;
Gx2[86] = 0.0;
;
Gx2[87] = 0.0;
;
Gx2[88] = 0.0;
;
Gx2[89] = 0.0;
;
Gx2[90] = 0.0;
;
Gx2[91] = 0.0;
;
Gx2[92] = 0.0;
;
Gx2[93] = 0.0;
;
Gx2[94] = 0.0;
;
Gx2[95] = 0.0;
;
Gx2[96] = 0.0;
;
Gx2[97] = 0.0;
;
Gx2[98] = 0.0;
;
Gx2[99] = 0.0;
;
Gx2[100] = 0.0;
;
Gx2[101] = 0.0;
;
Gx2[102] = 0.0;
;
Gx2[103] = 0.0;
;
Gx2[104] = 0.0;
;
Gx2[105] = 0.0;
;
Gx2[106] = 0.0;
;
Gx2[107] = 0.0;
;
Gx2[108] = 0.0;
;
Gx2[109] = 0.0;
;
Gx2[110] = 0.0;
;
Gx2[111] = 0.0;
;
Gx2[112] = 0.0;
;
Gx2[113] = 0.0;
;
Gx2[114] = 0.0;
;
Gx2[115] = 0.0;
;
Gx2[116] = 0.0;
;
Gx2[117] = 0.0;
;
Gx2[118] = 0.0;
;
Gx2[119] = 0.0;
;
Gx2[120] = 0.0;
;
Gx2[121] = 0.0;
;
Gx2[122] = 0.0;
;
Gx2[123] = 0.0;
;
Gx2[124] = 0.0;
;
Gx2[125] = 0.0;
;
Gx2[126] = 0.0;
;
Gx2[127] = 0.0;
;
Gx2[128] = 0.0;
;
Gx2[129] = 0.0;
;
Gx2[130] = 0.0;
;
Gx2[131] = 0.0;
;
Gx2[132] = 0.0;
;
Gx2[133] = 0.0;
;
Gx2[134] = 0.0;
;
Gx2[135] = 0.0;
;
Gx2[136] = 0.0;
;
Gx2[137] = 0.0;
;
Gx2[138] = 0.0;
;
Gx2[139] = 0.0;
;
Gx2[140] = 0.0;
;
Gx2[141] = 0.0;
;
Gx2[142] = 0.0;
;
Gx2[143] = 0.0;
;
}

void acado_multQ1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = 0.0;
;
Gu2[1] = 0.0;
;
Gu2[2] = 0.0;
;
Gu2[3] = 0.0;
;
Gu2[4] = 0.0;
;
Gu2[5] = 0.0;
;
Gu2[6] = 0.0;
;
Gu2[7] = 0.0;
;
Gu2[8] = + (real_t)2.5500000000000000e+01*Gu1[8];
Gu2[9] = + (real_t)2.5500000000000000e+01*Gu1[9];
Gu2[10] = + (real_t)2.5500000000000000e+01*Gu1[10];
Gu2[11] = + (real_t)2.5500000000000000e+01*Gu1[11];
Gu2[12] = + (real_t)1.4750000000000000e+02*Gu1[12];
Gu2[13] = + (real_t)1.4750000000000000e+02*Gu1[13];
Gu2[14] = + (real_t)1.4750000000000000e+02*Gu1[14];
Gu2[15] = + (real_t)1.4750000000000000e+02*Gu1[15];
Gu2[16] = + (real_t)1.4750000000000000e+02*Gu1[16];
Gu2[17] = + (real_t)1.4750000000000000e+02*Gu1[17];
Gu2[18] = + (real_t)1.4750000000000000e+02*Gu1[18];
Gu2[19] = + (real_t)1.4750000000000000e+02*Gu1[19];
Gu2[20] = + (real_t)1.4750000000000000e+02*Gu1[20];
Gu2[21] = + (real_t)1.4750000000000000e+02*Gu1[21];
Gu2[22] = + (real_t)1.4750000000000000e+02*Gu1[22];
Gu2[23] = + (real_t)1.4750000000000000e+02*Gu1[23];
Gu2[24] = 0.0;
;
Gu2[25] = 0.0;
;
Gu2[26] = 0.0;
;
Gu2[27] = 0.0;
;
Gu2[28] = 0.0;
;
Gu2[29] = 0.0;
;
Gu2[30] = 0.0;
;
Gu2[31] = 0.0;
;
Gu2[32] = 0.0;
;
Gu2[33] = 0.0;
;
Gu2[34] = 0.0;
;
Gu2[35] = 0.0;
;
Gu2[36] = 0.0;
;
Gu2[37] = 0.0;
;
Gu2[38] = 0.0;
;
Gu2[39] = 0.0;
;
Gu2[40] = 0.0;
;
Gu2[41] = 0.0;
;
Gu2[42] = 0.0;
;
Gu2[43] = 0.0;
;
Gu2[44] = 0.0;
;
Gu2[45] = 0.0;
;
Gu2[46] = 0.0;
;
Gu2[47] = 0.0;
;
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = 0.0;
;
Gu2[1] = 0.0;
;
Gu2[2] = 0.0;
;
Gu2[3] = 0.0;
;
Gu2[4] = 0.0;
;
Gu2[5] = 0.0;
;
Gu2[6] = 0.0;
;
Gu2[7] = 0.0;
;
Gu2[8] = + (real_t)2.5000000000000001e-02*Gu1[8];
Gu2[9] = + (real_t)2.5000000000000001e-02*Gu1[9];
Gu2[10] = + (real_t)2.5000000000000001e-02*Gu1[10];
Gu2[11] = + (real_t)2.5000000000000001e-02*Gu1[11];
Gu2[12] = + (real_t)2.5000000000000000e-01*Gu1[12];
Gu2[13] = + (real_t)2.5000000000000000e-01*Gu1[13];
Gu2[14] = + (real_t)2.5000000000000000e-01*Gu1[14];
Gu2[15] = + (real_t)2.5000000000000000e-01*Gu1[15];
Gu2[16] = + (real_t)2.5000000000000000e-01*Gu1[16];
Gu2[17] = + (real_t)2.5000000000000000e-01*Gu1[17];
Gu2[18] = + (real_t)2.5000000000000000e-01*Gu1[18];
Gu2[19] = + (real_t)2.5000000000000000e-01*Gu1[19];
Gu2[20] = + (real_t)2.5000000000000000e-01*Gu1[20];
Gu2[21] = + (real_t)2.5000000000000000e-01*Gu1[21];
Gu2[22] = + (real_t)2.5000000000000000e-01*Gu1[22];
Gu2[23] = + (real_t)2.5000000000000000e-01*Gu1[23];
Gu2[24] = 0.0;
;
Gu2[25] = 0.0;
;
Gu2[26] = 0.0;
;
Gu2[27] = 0.0;
;
Gu2[28] = 0.0;
;
Gu2[29] = 0.0;
;
Gu2[30] = 0.0;
;
Gu2[31] = 0.0;
;
Gu2[32] = 0.0;
;
Gu2[33] = 0.0;
;
Gu2[34] = 0.0;
;
Gu2[35] = 0.0;
;
Gu2[36] = 0.0;
;
Gu2[37] = 0.0;
;
Gu2[38] = 0.0;
;
Gu2[39] = 0.0;
;
Gu2[40] = 0.0;
;
Gu2[41] = 0.0;
;
Gu2[42] = 0.0;
;
Gu2[43] = 0.0;
;
Gu2[44] = 0.0;
;
Gu2[45] = 0.0;
;
Gu2[46] = 0.0;
;
Gu2[47] = 0.0;
;
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
g1[2] += 0.0;
;
g1[3] += 0.0;
;
}

void acado_condensePrep(  )
{
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_moveGxT( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.T );
acado_multGxd( acadoWorkspace.d, &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.d[ 12 ]) );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 144 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 48 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.E[ 96 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.d[ 24 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGx[ 288 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 144 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.E[ 192 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.E[ 240 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.d[ 36 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGx[ 432 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.E[ 288 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.E[ 336 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 384 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.E[ 432 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.d[ 48 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.evGx[ 576 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.E[ 480 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.E[ 528 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.E[ 576 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.E[ 624 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 192 ]), &(acadoWorkspace.E[ 672 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 720 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.d[ 60 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.evGx[ 720 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.E[ 720 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.E[ 768 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.E[ 816 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.E[ 864 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.E[ 912 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 240 ]), &(acadoWorkspace.E[ 960 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 864 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.d[ 72 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.evGx[ 864 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.E[ 1008 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.E[ 1056 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.E[ 1104 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.E[ 1152 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.E[ 1200 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.E[ 1248 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 288 ]), &(acadoWorkspace.E[ 1296 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 1008 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.d[ 84 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.evGx[ 1008 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.E[ 1344 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.E[ 1392 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.E[ 1440 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.E[ 1488 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.E[ 1536 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.E[ 1584 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.E[ 1632 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 336 ]), &(acadoWorkspace.E[ 1680 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 1152 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 84 ]), &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.d[ 96 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.evGx[ 1152 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.E[ 1728 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.E[ 1776 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.E[ 1824 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.E[ 1872 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.E[ 1920 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.E[ 1968 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.E[ 2016 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.E[ 2064 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 384 ]), &(acadoWorkspace.E[ 2112 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 96 ]), &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.d[ 108 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.evGx[ 1296 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.E[ 2160 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.E[ 2208 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.E[ 2256 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.E[ 2304 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.E[ 2352 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.E[ 2400 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.E[ 2448 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.E[ 2496 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.E[ 2544 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 432 ]), &(acadoWorkspace.E[ 2592 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 1440 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 108 ]), &(acadoWorkspace.evGx[ 1440 ]), &(acadoWorkspace.d[ 120 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.evGx[ 1440 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.E[ 2640 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.E[ 2688 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.E[ 2736 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.E[ 2784 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.E[ 2832 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.E[ 2880 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.E[ 2928 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.E[ 2976 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.E[ 3024 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.E[ 3072 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 480 ]), &(acadoWorkspace.E[ 3120 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 1584 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 120 ]), &(acadoWorkspace.evGx[ 1584 ]), &(acadoWorkspace.d[ 132 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 1440 ]), &(acadoWorkspace.evGx[ 1584 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.E[ 3168 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.E[ 3216 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.E[ 3264 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.E[ 3312 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.E[ 3360 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.E[ 3408 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.E[ 3456 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2976 ]), &(acadoWorkspace.E[ 3504 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 3024 ]), &(acadoWorkspace.E[ 3552 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.E[ 3600 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.E[ 3648 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 528 ]), &(acadoWorkspace.E[ 3696 ]) );

acado_multQ1Gu( acadoWorkspace.E, acadoWorkspace.QE );
acado_multQ1Gu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2448 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2496 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 2640 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2736 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2784 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2832 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2976 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 3024 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3072 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3120 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3168 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3216 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3264 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.QE[ 3312 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3360 ]), &(acadoWorkspace.QE[ 3360 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3408 ]), &(acadoWorkspace.QE[ 3408 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3456 ]), &(acadoWorkspace.QE[ 3456 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3504 ]), &(acadoWorkspace.QE[ 3504 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3552 ]), &(acadoWorkspace.QE[ 3552 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3600 ]), &(acadoWorkspace.QE[ 3600 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3648 ]), &(acadoWorkspace.QE[ 3648 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 3696 ]), &(acadoWorkspace.QE[ 3696 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 720 ]), &(acadoWorkspace.evGx[ 720 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1008 ]), &(acadoWorkspace.evGx[ 864 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1344 ]), &(acadoWorkspace.evGx[ 1008 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1728 ]), &(acadoWorkspace.evGx[ 1152 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 2160 ]), &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 2640 ]), &(acadoWorkspace.evGx[ 1440 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 3168 ]), &(acadoWorkspace.evGx[ 1584 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 336 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 528 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 768 ]), &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1056 ]), &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1392 ]), &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1776 ]), &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2208 ]), &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2688 ]), &(acadoWorkspace.evGx[ 1440 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3216 ]), &(acadoWorkspace.evGx[ 1584 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 384 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 576 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 816 ]), &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1104 ]), &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1440 ]), &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1824 ]), &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2256 ]), &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2736 ]), &(acadoWorkspace.evGx[ 1440 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3264 ]), &(acadoWorkspace.evGx[ 1584 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 624 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 864 ]), &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1152 ]), &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1488 ]), &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1872 ]), &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2304 ]), &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2784 ]), &(acadoWorkspace.evGx[ 1440 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3312 ]), &(acadoWorkspace.evGx[ 1584 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 672 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 912 ]), &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1200 ]), &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1536 ]), &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1920 ]), &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2352 ]), &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2832 ]), &(acadoWorkspace.evGx[ 1440 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3360 ]), &(acadoWorkspace.evGx[ 1584 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 240 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 960 ]), &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.H10[ 240 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1248 ]), &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.H10[ 240 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1584 ]), &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.H10[ 240 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1968 ]), &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.H10[ 240 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2400 ]), &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.H10[ 240 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2880 ]), &(acadoWorkspace.evGx[ 1440 ]), &(acadoWorkspace.H10[ 240 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3408 ]), &(acadoWorkspace.evGx[ 1584 ]), &(acadoWorkspace.H10[ 240 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 288 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1296 ]), &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.H10[ 288 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1632 ]), &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.H10[ 288 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2016 ]), &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.H10[ 288 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2448 ]), &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.H10[ 288 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2928 ]), &(acadoWorkspace.evGx[ 1440 ]), &(acadoWorkspace.H10[ 288 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3456 ]), &(acadoWorkspace.evGx[ 1584 ]), &(acadoWorkspace.H10[ 288 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 336 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1680 ]), &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.H10[ 336 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2064 ]), &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.H10[ 336 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2496 ]), &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.H10[ 336 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2976 ]), &(acadoWorkspace.evGx[ 1440 ]), &(acadoWorkspace.H10[ 336 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3504 ]), &(acadoWorkspace.evGx[ 1584 ]), &(acadoWorkspace.H10[ 336 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 384 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2112 ]), &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.H10[ 384 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2544 ]), &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.H10[ 384 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3024 ]), &(acadoWorkspace.evGx[ 1440 ]), &(acadoWorkspace.H10[ 384 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3552 ]), &(acadoWorkspace.evGx[ 1584 ]), &(acadoWorkspace.H10[ 384 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 432 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2592 ]), &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.H10[ 432 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3072 ]), &(acadoWorkspace.evGx[ 1440 ]), &(acadoWorkspace.H10[ 432 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3600 ]), &(acadoWorkspace.evGx[ 1584 ]), &(acadoWorkspace.H10[ 432 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 480 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3120 ]), &(acadoWorkspace.evGx[ 1440 ]), &(acadoWorkspace.H10[ 480 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3648 ]), &(acadoWorkspace.evGx[ 1584 ]), &(acadoWorkspace.H10[ 480 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 528 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 3696 ]), &(acadoWorkspace.evGx[ 1584 ]), &(acadoWorkspace.H10[ 528 ]) );

acado_setBlockH11_R1( 0, 0 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 2640 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3168 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3216 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 2736 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 2784 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 2832 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3360 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3408 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2448 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3456 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2496 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3504 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3552 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 3072 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3600 ]) );

acado_zeroBlockH11( 0, 10 );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QE[ 3120 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3648 ]) );

acado_zeroBlockH11( 0, 11 );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QE[ 3696 ]) );

acado_setBlockH11_R1( 1, 1 );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QE[ 2688 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3216 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QE[ 2736 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QE[ 2784 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QE[ 2832 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3360 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3408 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2448 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3456 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2496 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3504 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3552 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QE[ 3072 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3600 ]) );

acado_zeroBlockH11( 1, 10 );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QE[ 3120 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3648 ]) );

acado_zeroBlockH11( 1, 11 );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QE[ 3696 ]) );

acado_setBlockH11_R1( 2, 2 );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2736 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3264 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2784 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2832 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3360 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3408 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2448 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3456 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2496 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3504 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3552 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 3072 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3600 ]) );

acado_zeroBlockH11( 2, 10 );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QE[ 3120 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3648 ]) );

acado_zeroBlockH11( 2, 11 );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QE[ 3696 ]) );

acado_setBlockH11_R1( 3, 3 );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2784 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.QE[ 3312 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2832 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.QE[ 3360 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.QE[ 3408 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2448 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.QE[ 3456 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2496 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.QE[ 3504 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.QE[ 3552 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 3072 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.QE[ 3600 ]) );

acado_zeroBlockH11( 3, 10 );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QE[ 3120 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.QE[ 3648 ]) );

acado_zeroBlockH11( 3, 11 );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.QE[ 3696 ]) );

acado_setBlockH11_R1( 4, 4 );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2832 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 3360 ]), &(acadoWorkspace.QE[ 3360 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 3360 ]), &(acadoWorkspace.QE[ 3408 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2448 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 3360 ]), &(acadoWorkspace.QE[ 3456 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2496 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 3360 ]), &(acadoWorkspace.QE[ 3504 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 3360 ]), &(acadoWorkspace.QE[ 3552 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 3072 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 3360 ]), &(acadoWorkspace.QE[ 3600 ]) );

acado_zeroBlockH11( 4, 10 );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QE[ 3120 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 3360 ]), &(acadoWorkspace.QE[ 3648 ]) );

acado_zeroBlockH11( 4, 11 );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 3360 ]), &(acadoWorkspace.QE[ 3696 ]) );

acado_setBlockH11_R1( 5, 5 );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 2880 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 3408 ]), &(acadoWorkspace.QE[ 3408 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2448 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 3408 ]), &(acadoWorkspace.QE[ 3456 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2496 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 3408 ]), &(acadoWorkspace.QE[ 3504 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 3408 ]), &(acadoWorkspace.QE[ 3552 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 3072 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 3408 ]), &(acadoWorkspace.QE[ 3600 ]) );

acado_zeroBlockH11( 5, 10 );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QE[ 3120 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 3408 ]), &(acadoWorkspace.QE[ 3648 ]) );

acado_zeroBlockH11( 5, 11 );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 3408 ]), &(acadoWorkspace.QE[ 3696 ]) );

acado_setBlockH11_R1( 6, 6 );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2448 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.QE[ 2928 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 3456 ]), &(acadoWorkspace.QE[ 3456 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2496 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 3456 ]), &(acadoWorkspace.QE[ 3504 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 3456 ]), &(acadoWorkspace.QE[ 3552 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.QE[ 3072 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 3456 ]), &(acadoWorkspace.QE[ 3600 ]) );

acado_zeroBlockH11( 6, 10 );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.QE[ 3120 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 3456 ]), &(acadoWorkspace.QE[ 3648 ]) );

acado_zeroBlockH11( 6, 11 );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 3456 ]), &(acadoWorkspace.QE[ 3696 ]) );

acado_setBlockH11_R1( 7, 7 );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2496 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 2976 ]), &(acadoWorkspace.QE[ 2976 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 3504 ]), &(acadoWorkspace.QE[ 3504 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 2976 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 3504 ]), &(acadoWorkspace.QE[ 3552 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 2976 ]), &(acadoWorkspace.QE[ 3072 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 3504 ]), &(acadoWorkspace.QE[ 3600 ]) );

acado_zeroBlockH11( 7, 10 );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 2976 ]), &(acadoWorkspace.QE[ 3120 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 3504 ]), &(acadoWorkspace.QE[ 3648 ]) );

acado_zeroBlockH11( 7, 11 );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 3504 ]), &(acadoWorkspace.QE[ 3696 ]) );

acado_setBlockH11_R1( 8, 8 );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QE[ 2544 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 3024 ]), &(acadoWorkspace.QE[ 3024 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 3552 ]), &(acadoWorkspace.QE[ 3552 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 3024 ]), &(acadoWorkspace.QE[ 3072 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 3552 ]), &(acadoWorkspace.QE[ 3600 ]) );

acado_zeroBlockH11( 8, 10 );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 3024 ]), &(acadoWorkspace.QE[ 3120 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 3552 ]), &(acadoWorkspace.QE[ 3648 ]) );

acado_zeroBlockH11( 8, 11 );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 3552 ]), &(acadoWorkspace.QE[ 3696 ]) );

acado_setBlockH11_R1( 9, 9 );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.QE[ 2592 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3072 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 3600 ]), &(acadoWorkspace.QE[ 3600 ]) );

acado_zeroBlockH11( 9, 10 );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QE[ 3120 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 3600 ]), &(acadoWorkspace.QE[ 3648 ]) );

acado_zeroBlockH11( 9, 11 );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 3600 ]), &(acadoWorkspace.QE[ 3696 ]) );

acado_setBlockH11_R1( 10, 10 );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QE[ 3120 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 3648 ]), &(acadoWorkspace.QE[ 3648 ]) );

acado_zeroBlockH11( 10, 11 );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 3648 ]), &(acadoWorkspace.QE[ 3696 ]) );

acado_setBlockH11_R1( 11, 11 );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 3696 ]), &(acadoWorkspace.QE[ 3696 ]) );


acado_copyHTH( 1, 0 );
acado_copyHTH( 2, 0 );
acado_copyHTH( 2, 1 );
acado_copyHTH( 3, 0 );
acado_copyHTH( 3, 1 );
acado_copyHTH( 3, 2 );
acado_copyHTH( 4, 0 );
acado_copyHTH( 4, 1 );
acado_copyHTH( 4, 2 );
acado_copyHTH( 4, 3 );
acado_copyHTH( 5, 0 );
acado_copyHTH( 5, 1 );
acado_copyHTH( 5, 2 );
acado_copyHTH( 5, 3 );
acado_copyHTH( 5, 4 );
acado_copyHTH( 6, 0 );
acado_copyHTH( 6, 1 );
acado_copyHTH( 6, 2 );
acado_copyHTH( 6, 3 );
acado_copyHTH( 6, 4 );
acado_copyHTH( 6, 5 );
acado_copyHTH( 7, 0 );
acado_copyHTH( 7, 1 );
acado_copyHTH( 7, 2 );
acado_copyHTH( 7, 3 );
acado_copyHTH( 7, 4 );
acado_copyHTH( 7, 5 );
acado_copyHTH( 7, 6 );
acado_copyHTH( 8, 0 );
acado_copyHTH( 8, 1 );
acado_copyHTH( 8, 2 );
acado_copyHTH( 8, 3 );
acado_copyHTH( 8, 4 );
acado_copyHTH( 8, 5 );
acado_copyHTH( 8, 6 );
acado_copyHTH( 8, 7 );
acado_copyHTH( 9, 0 );
acado_copyHTH( 9, 1 );
acado_copyHTH( 9, 2 );
acado_copyHTH( 9, 3 );
acado_copyHTH( 9, 4 );
acado_copyHTH( 9, 5 );
acado_copyHTH( 9, 6 );
acado_copyHTH( 9, 7 );
acado_copyHTH( 9, 8 );
acado_copyHTH( 10, 0 );
acado_copyHTH( 10, 1 );
acado_copyHTH( 10, 2 );
acado_copyHTH( 10, 3 );
acado_copyHTH( 10, 4 );
acado_copyHTH( 10, 5 );
acado_copyHTH( 10, 6 );
acado_copyHTH( 10, 7 );
acado_copyHTH( 10, 8 );
acado_copyHTH( 10, 9 );
acado_copyHTH( 11, 0 );
acado_copyHTH( 11, 1 );
acado_copyHTH( 11, 2 );
acado_copyHTH( 11, 3 );
acado_copyHTH( 11, 4 );
acado_copyHTH( 11, 5 );
acado_copyHTH( 11, 6 );
acado_copyHTH( 11, 7 );
acado_copyHTH( 11, 8 );
acado_copyHTH( 11, 9 );
acado_copyHTH( 11, 10 );

acado_multQ1d( acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.Qd[ 12 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.Qd[ 24 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.Qd[ 36 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.Qd[ 48 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.Qd[ 72 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 84 ]), &(acadoWorkspace.Qd[ 84 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 96 ]), &(acadoWorkspace.Qd[ 96 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 108 ]), &(acadoWorkspace.Qd[ 108 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 120 ]), &(acadoWorkspace.Qd[ 120 ]) );
acado_multQN1d( &(acadoWorkspace.d[ 132 ]), &(acadoWorkspace.Qd[ 132 ]) );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 48 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 144 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 288 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 480 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 720 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1008 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1344 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1728 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 2160 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 2640 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 3168 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 336 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 528 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 768 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1056 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1392 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1776 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2208 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2688 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3216 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 384 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 576 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 816 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1104 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1440 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1824 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2256 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2736 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3264 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 624 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 864 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1152 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1488 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1872 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2304 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2784 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3312 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 672 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 912 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1200 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1536 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1920 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2352 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2832 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3360 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 960 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1248 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1584 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1968 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2400 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2880 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3408 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1296 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1632 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2016 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2448 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2928 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3456 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1680 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2064 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2496 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2976 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3504 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2112 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2544 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3024 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3552 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2592 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3072 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3600 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3120 ]), &(acadoWorkspace.g[ 40 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3648 ]), &(acadoWorkspace.g[ 40 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 3696 ]), &(acadoWorkspace.g[ 44 ]) );
acadoWorkspace.lb[0] = (real_t)1.5000000000000000e+01 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-5.0000000000000003e-02 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)1.5000000000000000e+01 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-5.0000000000000003e-02 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)1.5000000000000000e+01 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-5.0000000000000003e-02 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)1.5000000000000000e+01 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-5.0000000000000003e-02 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)1.5000000000000000e+01 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-5.0000000000000003e-02 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)1.5000000000000000e+01 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-5.0000000000000003e-02 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)1.5000000000000000e+01 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-5.0000000000000003e-02 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)1.5000000000000000e+01 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-5.0000000000000003e-02 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)1.5000000000000000e+01 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-5.0000000000000003e-02 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)1.5000000000000000e+01 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-5.0000000000000003e-02 - acadoVariables.u[39];
acadoWorkspace.lb[40] = (real_t)1.5000000000000000e+01 - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[41];
acadoWorkspace.lb[42] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[42];
acadoWorkspace.lb[43] = (real_t)-5.0000000000000003e-02 - acadoVariables.u[43];
acadoWorkspace.lb[44] = (real_t)1.5000000000000000e+01 - acadoVariables.u[44];
acadoWorkspace.lb[45] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[45];
acadoWorkspace.lb[46] = (real_t)-1.0000000000000000e-02 - acadoVariables.u[46];
acadoWorkspace.lb[47] = (real_t)-5.0000000000000003e-02 - acadoVariables.u[47];
acadoWorkspace.ub[0] = (real_t)2.5000000000000000e+01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)1.0000000000000000e-02 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e-02 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)5.0000000000000003e-02 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)2.5000000000000000e+01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)1.0000000000000000e-02 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)1.0000000000000000e-02 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)5.0000000000000003e-02 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)2.5000000000000000e+01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)1.0000000000000000e-02 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e-02 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)5.0000000000000003e-02 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)2.5000000000000000e+01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)1.0000000000000000e-02 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e-02 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)5.0000000000000003e-02 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)2.5000000000000000e+01 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)1.0000000000000000e-02 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)1.0000000000000000e-02 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)5.0000000000000003e-02 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)2.5000000000000000e+01 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)1.0000000000000000e-02 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)1.0000000000000000e-02 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)5.0000000000000003e-02 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)2.5000000000000000e+01 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)1.0000000000000000e-02 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)1.0000000000000000e-02 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)5.0000000000000003e-02 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)2.5000000000000000e+01 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)1.0000000000000000e-02 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)1.0000000000000000e-02 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)5.0000000000000003e-02 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)2.5000000000000000e+01 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)1.0000000000000000e-02 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)1.0000000000000000e-02 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)5.0000000000000003e-02 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)2.5000000000000000e+01 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)1.0000000000000000e-02 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)1.0000000000000000e-02 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)5.0000000000000003e-02 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)2.5000000000000000e+01 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)1.0000000000000000e-02 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)1.0000000000000000e-02 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)5.0000000000000003e-02 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)2.5000000000000000e+01 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)1.0000000000000000e-02 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)1.0000000000000000e-02 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)5.0000000000000003e-02 - acadoVariables.u[47];

}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
acadoWorkspace.Dx0[9] = acadoVariables.x0[9] - acadoVariables.x[9];
acadoWorkspace.Dx0[10] = acadoVariables.x0[10] - acadoVariables.x[10];
acadoWorkspace.Dx0[11] = acadoVariables.x0[11] - acadoVariables.x[11];

acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.Dy[40] -= acadoVariables.y[40];
acadoWorkspace.Dy[41] -= acadoVariables.y[41];
acadoWorkspace.Dy[42] -= acadoVariables.y[42];
acadoWorkspace.Dy[43] -= acadoVariables.y[43];
acadoWorkspace.Dy[44] -= acadoVariables.y[44];
acadoWorkspace.Dy[45] -= acadoVariables.y[45];
acadoWorkspace.Dy[46] -= acadoVariables.y[46];
acadoWorkspace.Dy[47] -= acadoVariables.y[47];
acadoWorkspace.Dy[48] -= acadoVariables.y[48];
acadoWorkspace.Dy[49] -= acadoVariables.y[49];
acadoWorkspace.Dy[50] -= acadoVariables.y[50];
acadoWorkspace.Dy[51] -= acadoVariables.y[51];
acadoWorkspace.Dy[52] -= acadoVariables.y[52];
acadoWorkspace.Dy[53] -= acadoVariables.y[53];
acadoWorkspace.Dy[54] -= acadoVariables.y[54];
acadoWorkspace.Dy[55] -= acadoVariables.y[55];
acadoWorkspace.Dy[56] -= acadoVariables.y[56];
acadoWorkspace.Dy[57] -= acadoVariables.y[57];
acadoWorkspace.Dy[58] -= acadoVariables.y[58];
acadoWorkspace.Dy[59] -= acadoVariables.y[59];
acadoWorkspace.Dy[60] -= acadoVariables.y[60];
acadoWorkspace.Dy[61] -= acadoVariables.y[61];
acadoWorkspace.Dy[62] -= acadoVariables.y[62];
acadoWorkspace.Dy[63] -= acadoVariables.y[63];
acadoWorkspace.Dy[64] -= acadoVariables.y[64];
acadoWorkspace.Dy[65] -= acadoVariables.y[65];
acadoWorkspace.Dy[66] -= acadoVariables.y[66];
acadoWorkspace.Dy[67] -= acadoVariables.y[67];
acadoWorkspace.Dy[68] -= acadoVariables.y[68];
acadoWorkspace.Dy[69] -= acadoVariables.y[69];
acadoWorkspace.Dy[70] -= acadoVariables.y[70];
acadoWorkspace.Dy[71] -= acadoVariables.y[71];
acadoWorkspace.Dy[72] -= acadoVariables.y[72];
acadoWorkspace.Dy[73] -= acadoVariables.y[73];
acadoWorkspace.Dy[74] -= acadoVariables.y[74];
acadoWorkspace.Dy[75] -= acadoVariables.y[75];
acadoWorkspace.Dy[76] -= acadoVariables.y[76];
acadoWorkspace.Dy[77] -= acadoVariables.y[77];
acadoWorkspace.Dy[78] -= acadoVariables.y[78];
acadoWorkspace.Dy[79] -= acadoVariables.y[79];
acadoWorkspace.Dy[80] -= acadoVariables.y[80];
acadoWorkspace.Dy[81] -= acadoVariables.y[81];
acadoWorkspace.Dy[82] -= acadoVariables.y[82];
acadoWorkspace.Dy[83] -= acadoVariables.y[83];
acadoWorkspace.Dy[84] -= acadoVariables.y[84];
acadoWorkspace.Dy[85] -= acadoVariables.y[85];
acadoWorkspace.Dy[86] -= acadoVariables.y[86];
acadoWorkspace.Dy[87] -= acadoVariables.y[87];
acadoWorkspace.Dy[88] -= acadoVariables.y[88];
acadoWorkspace.Dy[89] -= acadoVariables.y[89];
acadoWorkspace.Dy[90] -= acadoVariables.y[90];
acadoWorkspace.Dy[91] -= acadoVariables.y[91];
acadoWorkspace.Dy[92] -= acadoVariables.y[92];
acadoWorkspace.Dy[93] -= acadoVariables.y[93];
acadoWorkspace.Dy[94] -= acadoVariables.y[94];
acadoWorkspace.Dy[95] -= acadoVariables.y[95];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.g[ 44 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.QDy[ 132 ]) );

acadoWorkspace.QDy[144] = 0.0;
;
acadoWorkspace.QDy[145] = 0.0;
;
acadoWorkspace.QDy[146] = + (real_t)2.5000000000000001e-02*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[147] = + (real_t)2.5000000000000000e-01*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[148] = + (real_t)2.5000000000000000e-01*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[149] = + (real_t)2.5000000000000000e-01*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[150] = 0.0;
;
acadoWorkspace.QDy[151] = 0.0;
;
acadoWorkspace.QDy[152] = 0.0;
;
acadoWorkspace.QDy[153] = 0.0;
;
acadoWorkspace.QDy[154] = 0.0;
;
acadoWorkspace.QDy[155] = 0.0;
;

for (lRun1 = 0; lRun1 < 144; ++lRun1)
acadoWorkspace.QDy[lRun1 + 12] += acadoWorkspace.Qd[lRun1];


acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QDy[ 84 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QDy[ 96 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QDy[ 120 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 2640 ]), &(acadoWorkspace.QDy[ 132 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 3168 ]), &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.QDy[ 132 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.QDy[ 144 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.QDy[ 132 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.QDy[ 144 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.QDy[ 132 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.QDy[ 144 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.QDy[ 132 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3360 ]), &(acadoWorkspace.QDy[ 144 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.QDy[ 132 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3408 ]), &(acadoWorkspace.QDy[ 144 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.QDy[ 132 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3456 ]), &(acadoWorkspace.QDy[ 144 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2976 ]), &(acadoWorkspace.QDy[ 132 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3504 ]), &(acadoWorkspace.QDy[ 144 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3024 ]), &(acadoWorkspace.QDy[ 132 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3552 ]), &(acadoWorkspace.QDy[ 144 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.QDy[ 132 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3600 ]), &(acadoWorkspace.QDy[ 144 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.QDy[ 132 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3648 ]), &(acadoWorkspace.QDy[ 144 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 3696 ]), &(acadoWorkspace.QDy[ 144 ]), &(acadoWorkspace.g[ 44 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[1] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[2] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[3] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[4] += + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[5] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[6] += + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[7] += + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[8] += + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[9] += + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[10] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[11] += + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[12] += + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[13] += + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[160]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[161]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[162]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[163]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[164]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[165]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[166]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[167]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[14] += + acadoWorkspace.H10[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[171]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[172]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[173]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[174]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[175]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[176]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[177]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[178]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[179]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[15] += + acadoWorkspace.H10[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[185]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[186]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[187]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[188]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[189]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[190]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[191]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[16] += + acadoWorkspace.H10[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[195]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[196]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[197]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[198]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[199]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[200]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[201]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[202]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[203]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[17] += + acadoWorkspace.H10[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[207]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[208]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[209]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[210]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[211]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[212]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[213]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[214]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[215]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[18] += + acadoWorkspace.H10[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[220]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[221]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[222]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[223]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[224]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[225]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[226]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[227]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[19] += + acadoWorkspace.H10[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[231]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[232]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[233]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[234]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[235]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[236]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[237]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[238]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[239]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[20] += + acadoWorkspace.H10[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[244]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[245]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[246]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[247]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[248]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[249]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[250]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[251]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[21] += + acadoWorkspace.H10[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[255]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[256]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[257]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[258]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[259]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[260]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[261]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[262]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[263]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[22] += + acadoWorkspace.H10[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[267]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[268]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[269]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[270]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[271]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[272]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[273]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[274]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[275]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[23] += + acadoWorkspace.H10[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[279]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[280]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[281]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[282]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[283]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[284]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[285]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[286]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[287]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[24] += + acadoWorkspace.H10[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[291]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[292]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[293]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[294]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[295]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[296]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[297]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[298]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[299]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[25] += + acadoWorkspace.H10[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[304]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[305]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[306]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[307]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[308]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[309]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[310]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[311]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[26] += + acadoWorkspace.H10[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[315]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[316]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[317]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[318]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[319]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[320]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[321]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[322]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[323]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[27] += + acadoWorkspace.H10[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[326]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[327]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[328]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[329]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[330]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[331]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[332]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[333]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[334]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[335]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[28] += + acadoWorkspace.H10[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[339]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[340]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[341]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[342]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[343]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[344]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[345]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[346]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[347]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[29] += + acadoWorkspace.H10[348]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[349]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[350]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[351]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[352]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[353]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[354]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[355]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[356]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[357]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[358]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[359]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[30] += + acadoWorkspace.H10[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[364]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[365]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[366]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[367]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[368]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[369]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[370]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[371]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[31] += + acadoWorkspace.H10[372]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[373]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[374]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[375]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[376]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[377]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[378]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[379]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[380]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[381]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[382]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[383]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[32] += + acadoWorkspace.H10[384]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[385]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[386]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[387]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[388]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[389]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[390]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[391]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[392]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[393]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[394]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[395]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[33] += + acadoWorkspace.H10[396]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[397]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[398]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[399]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[400]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[401]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[402]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[403]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[404]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[405]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[406]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[407]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[34] += + acadoWorkspace.H10[408]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[409]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[410]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[411]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[412]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[413]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[414]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[415]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[416]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[417]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[418]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[419]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[35] += + acadoWorkspace.H10[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[424]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[425]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[426]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[427]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[428]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[429]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[430]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[431]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[36] += + acadoWorkspace.H10[432]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[433]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[434]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[435]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[436]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[437]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[438]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[439]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[440]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[441]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[442]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[443]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[37] += + acadoWorkspace.H10[444]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[445]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[446]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[447]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[448]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[449]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[450]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[451]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[452]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[453]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[454]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[455]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[38] += + acadoWorkspace.H10[456]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[457]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[458]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[459]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[460]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[461]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[462]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[463]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[464]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[465]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[466]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[467]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[39] += + acadoWorkspace.H10[468]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[469]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[470]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[471]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[472]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[473]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[474]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[475]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[476]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[477]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[478]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[479]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[40] += + acadoWorkspace.H10[480]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[481]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[482]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[483]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[484]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[485]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[486]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[487]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[488]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[489]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[490]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[491]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[41] += + acadoWorkspace.H10[492]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[493]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[494]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[495]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[496]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[497]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[498]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[499]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[500]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[501]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[502]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[503]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[42] += + acadoWorkspace.H10[504]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[505]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[506]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[507]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[508]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[509]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[510]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[511]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[512]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[513]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[514]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[515]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[43] += + acadoWorkspace.H10[516]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[517]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[518]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[519]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[520]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[521]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[522]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[523]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[524]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[525]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[526]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[527]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[44] += + acadoWorkspace.H10[528]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[529]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[530]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[531]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[532]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[533]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[534]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[535]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[536]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[537]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[538]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[539]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[45] += + acadoWorkspace.H10[540]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[541]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[542]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[543]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[544]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[545]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[546]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[547]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[548]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[549]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[550]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[551]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[46] += + acadoWorkspace.H10[552]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[553]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[554]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[555]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[556]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[557]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[558]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[559]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[560]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[561]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[562]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[563]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[47] += + acadoWorkspace.H10[564]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[565]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[566]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[567]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[568]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[569]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[570]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[571]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[572]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[573]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[574]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[575]*acadoWorkspace.Dx0[11];

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];
acadoVariables.x[4] += acadoWorkspace.Dx0[4];
acadoVariables.x[5] += acadoWorkspace.Dx0[5];
acadoVariables.x[6] += acadoWorkspace.Dx0[6];
acadoVariables.x[7] += acadoWorkspace.Dx0[7];
acadoVariables.x[8] += acadoWorkspace.Dx0[8];
acadoVariables.x[9] += acadoWorkspace.Dx0[9];
acadoVariables.x[10] += acadoWorkspace.Dx0[10];
acadoVariables.x[11] += acadoWorkspace.Dx0[11];

acadoVariables.x[12] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[0];
acadoVariables.x[13] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[1];
acadoVariables.x[14] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[2];
acadoVariables.x[15] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[3];
acadoVariables.x[16] += + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[4];
acadoVariables.x[17] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[5];
acadoVariables.x[18] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[6];
acadoVariables.x[19] += + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[7];
acadoVariables.x[20] += + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[8];
acadoVariables.x[21] += + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[9];
acadoVariables.x[22] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[10];
acadoVariables.x[23] += + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[11];
acadoVariables.x[24] += + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[12];
acadoVariables.x[25] += + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[13];
acadoVariables.x[26] += + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[14];
acadoVariables.x[27] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[15];
acadoVariables.x[28] += + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[16];
acadoVariables.x[29] += + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[17];
acadoVariables.x[30] += + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[18];
acadoVariables.x[31] += + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[19];
acadoVariables.x[32] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[20];
acadoVariables.x[33] += + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[21];
acadoVariables.x[34] += + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[22];
acadoVariables.x[35] += + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[23];
acadoVariables.x[36] += + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[24];
acadoVariables.x[37] += + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[25];
acadoVariables.x[38] += + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[26];
acadoVariables.x[39] += + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[326]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[327]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[328]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[329]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[330]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[331]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[332]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[333]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[334]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[335]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[27];
acadoVariables.x[40] += + acadoWorkspace.evGx[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[339]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[28];
acadoVariables.x[41] += + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[350]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[351]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[352]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[353]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[354]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[355]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[356]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[357]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[358]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[359]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[29];
acadoVariables.x[42] += + acadoWorkspace.evGx[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[364]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[365]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[366]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[367]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[30];
acadoVariables.x[43] += + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[375]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[376]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[377]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[378]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[379]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[380]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[381]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[382]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[383]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[31];
acadoVariables.x[44] += + acadoWorkspace.evGx[384]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[385]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[386]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[387]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[388]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[389]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[392]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[393]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[394]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[32];
acadoVariables.x[45] += + acadoWorkspace.evGx[396]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[397]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[398]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[399]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[400]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[401]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[402]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[403]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[404]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[405]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[406]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[407]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[33];
acadoVariables.x[46] += + acadoWorkspace.evGx[408]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[409]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[410]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[411]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[412]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[413]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[414]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[415]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[416]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[417]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[418]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[419]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[34];
acadoVariables.x[47] += + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[424]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[425]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[426]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[427]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[428]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[429]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[430]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[431]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[35];
acadoVariables.x[48] += + acadoWorkspace.evGx[432]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[433]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[434]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[435]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[436]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[437]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[438]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[439]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[440]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[441]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[442]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[443]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[36];
acadoVariables.x[49] += + acadoWorkspace.evGx[444]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[445]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[446]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[447]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[450]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[451]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[452]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[453]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[454]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[455]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[37];
acadoVariables.x[50] += + acadoWorkspace.evGx[456]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[457]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[458]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[459]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[460]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[461]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[462]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[463]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[464]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[465]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[466]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[467]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[38];
acadoVariables.x[51] += + acadoWorkspace.evGx[468]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[469]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[470]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[471]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[472]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[473]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[474]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[475]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[476]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[477]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[478]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[479]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[39];
acadoVariables.x[52] += + acadoWorkspace.evGx[480]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[481]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[482]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[483]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[484]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[485]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[486]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[487]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[488]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[489]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[490]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[491]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[40];
acadoVariables.x[53] += + acadoWorkspace.evGx[492]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[493]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[494]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[495]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[496]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[497]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[498]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[499]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[500]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[501]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[502]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[503]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[41];
acadoVariables.x[54] += + acadoWorkspace.evGx[504]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[505]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[506]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[507]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[508]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[509]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[510]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[511]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[512]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[513]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[514]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[515]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[42];
acadoVariables.x[55] += + acadoWorkspace.evGx[516]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[517]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[518]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[519]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[520]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[521]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[522]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[523]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[524]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[525]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[526]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[527]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[43];
acadoVariables.x[56] += + acadoWorkspace.evGx[528]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[529]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[530]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[531]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[532]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[533]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[534]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[535]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[536]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[537]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[538]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[539]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[44];
acadoVariables.x[57] += + acadoWorkspace.evGx[540]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[541]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[542]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[543]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[544]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[545]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[546]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[547]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[548]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[549]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[550]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[551]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[45];
acadoVariables.x[58] += + acadoWorkspace.evGx[552]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[553]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[554]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[555]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[556]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[557]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[558]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[559]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[560]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[561]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[562]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[563]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[46];
acadoVariables.x[59] += + acadoWorkspace.evGx[564]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[565]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[566]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[567]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[568]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[569]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[570]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[571]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[572]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[573]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[574]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[575]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[47];
acadoVariables.x[60] += + acadoWorkspace.evGx[576]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[577]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[578]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[579]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[580]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[581]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[582]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[583]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[584]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[585]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[586]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[587]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[48];
acadoVariables.x[61] += + acadoWorkspace.evGx[588]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[589]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[590]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[591]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[592]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[593]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[594]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[595]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[596]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[597]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[598]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[599]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[49];
acadoVariables.x[62] += + acadoWorkspace.evGx[600]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[601]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[602]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[603]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[604]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[605]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[606]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[607]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[608]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[609]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[610]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[611]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[50];
acadoVariables.x[63] += + acadoWorkspace.evGx[612]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[613]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[614]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[615]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[616]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[617]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[618]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[619]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[620]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[621]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[622]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[623]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[51];
acadoVariables.x[64] += + acadoWorkspace.evGx[624]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[625]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[626]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[627]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[628]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[629]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[630]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[631]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[632]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[633]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[634]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[635]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[52];
acadoVariables.x[65] += + acadoWorkspace.evGx[636]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[637]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[638]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[639]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[640]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[641]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[642]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[643]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[644]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[645]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[646]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[647]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[53];
acadoVariables.x[66] += + acadoWorkspace.evGx[648]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[649]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[650]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[651]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[652]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[653]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[654]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[655]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[656]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[657]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[658]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[659]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[54];
acadoVariables.x[67] += + acadoWorkspace.evGx[660]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[661]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[662]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[663]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[664]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[665]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[666]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[667]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[668]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[669]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[670]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[671]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[55];
acadoVariables.x[68] += + acadoWorkspace.evGx[672]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[673]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[674]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[675]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[676]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[677]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[678]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[679]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[680]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[681]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[682]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[683]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[56];
acadoVariables.x[69] += + acadoWorkspace.evGx[684]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[685]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[686]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[687]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[688]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[689]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[690]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[691]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[692]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[693]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[694]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[695]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[57];
acadoVariables.x[70] += + acadoWorkspace.evGx[696]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[697]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[698]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[699]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[700]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[701]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[702]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[703]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[704]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[705]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[706]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[707]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[58];
acadoVariables.x[71] += + acadoWorkspace.evGx[708]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[709]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[710]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[711]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[712]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[713]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[714]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[715]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[716]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[717]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[718]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[719]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[59];
acadoVariables.x[72] += + acadoWorkspace.evGx[720]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[721]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[722]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[723]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[724]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[725]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[726]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[727]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[728]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[729]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[730]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[731]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[60];
acadoVariables.x[73] += + acadoWorkspace.evGx[732]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[733]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[734]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[735]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[736]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[737]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[738]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[739]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[740]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[741]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[742]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[743]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[61];
acadoVariables.x[74] += + acadoWorkspace.evGx[744]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[745]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[746]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[747]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[748]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[749]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[750]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[751]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[752]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[753]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[754]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[755]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[62];
acadoVariables.x[75] += + acadoWorkspace.evGx[756]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[757]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[758]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[759]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[760]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[761]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[762]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[763]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[764]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[765]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[766]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[767]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[63];
acadoVariables.x[76] += + acadoWorkspace.evGx[768]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[769]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[770]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[771]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[772]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[773]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[774]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[775]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[776]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[777]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[778]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[779]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[64];
acadoVariables.x[77] += + acadoWorkspace.evGx[780]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[781]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[782]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[783]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[784]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[785]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[786]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[787]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[788]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[789]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[790]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[791]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[65];
acadoVariables.x[78] += + acadoWorkspace.evGx[792]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[793]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[794]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[795]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[796]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[797]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[798]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[799]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[800]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[801]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[802]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[803]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[66];
acadoVariables.x[79] += + acadoWorkspace.evGx[804]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[805]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[806]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[807]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[808]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[809]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[810]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[811]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[812]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[813]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[814]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[815]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[67];
acadoVariables.x[80] += + acadoWorkspace.evGx[816]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[817]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[818]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[819]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[820]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[821]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[822]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[823]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[824]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[825]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[826]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[827]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[68];
acadoVariables.x[81] += + acadoWorkspace.evGx[828]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[829]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[830]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[831]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[832]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[833]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[834]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[835]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[836]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[837]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[838]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[839]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[69];
acadoVariables.x[82] += + acadoWorkspace.evGx[840]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[841]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[842]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[843]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[844]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[845]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[846]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[847]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[848]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[849]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[850]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[851]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[70];
acadoVariables.x[83] += + acadoWorkspace.evGx[852]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[853]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[854]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[855]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[856]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[857]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[858]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[859]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[860]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[861]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[862]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[863]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[71];
acadoVariables.x[84] += + acadoWorkspace.evGx[864]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[865]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[866]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[867]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[868]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[869]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[870]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[871]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[872]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[873]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[874]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[875]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[72];
acadoVariables.x[85] += + acadoWorkspace.evGx[876]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[877]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[878]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[879]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[880]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[881]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[882]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[883]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[884]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[885]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[886]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[887]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[73];
acadoVariables.x[86] += + acadoWorkspace.evGx[888]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[889]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[890]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[891]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[892]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[893]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[894]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[895]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[896]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[897]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[898]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[899]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[74];
acadoVariables.x[87] += + acadoWorkspace.evGx[900]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[901]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[902]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[903]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[904]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[905]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[906]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[907]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[908]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[909]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[910]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[911]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[75];
acadoVariables.x[88] += + acadoWorkspace.evGx[912]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[913]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[914]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[915]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[916]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[917]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[918]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[919]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[920]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[921]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[922]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[923]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[76];
acadoVariables.x[89] += + acadoWorkspace.evGx[924]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[925]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[926]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[927]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[928]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[929]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[930]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[931]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[932]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[933]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[934]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[935]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[77];
acadoVariables.x[90] += + acadoWorkspace.evGx[936]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[937]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[938]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[939]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[940]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[941]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[942]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[943]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[944]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[945]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[946]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[947]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[78];
acadoVariables.x[91] += + acadoWorkspace.evGx[948]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[949]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[950]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[951]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[952]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[953]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[954]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[955]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[956]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[957]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[958]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[959]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[79];
acadoVariables.x[92] += + acadoWorkspace.evGx[960]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[961]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[962]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[963]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[964]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[965]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[966]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[967]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[968]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[969]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[970]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[971]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[80];
acadoVariables.x[93] += + acadoWorkspace.evGx[972]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[973]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[974]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[975]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[976]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[977]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[978]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[979]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[980]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[981]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[982]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[983]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[81];
acadoVariables.x[94] += + acadoWorkspace.evGx[984]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[985]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[986]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[987]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[988]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[989]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[990]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[991]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[992]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[993]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[994]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[995]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[82];
acadoVariables.x[95] += + acadoWorkspace.evGx[996]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[997]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[998]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[999]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1000]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1001]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1002]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1003]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1004]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1005]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1006]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1007]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[83];
acadoVariables.x[96] += + acadoWorkspace.evGx[1008]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1009]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1010]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1011]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1012]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1013]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1014]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1015]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1016]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1017]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1018]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1019]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[84];
acadoVariables.x[97] += + acadoWorkspace.evGx[1020]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1021]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1022]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1023]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1024]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1025]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1026]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1027]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1028]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1029]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1030]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1031]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[85];
acadoVariables.x[98] += + acadoWorkspace.evGx[1032]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1033]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1034]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1035]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1036]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1037]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1038]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1039]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1040]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1041]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1042]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1043]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[86];
acadoVariables.x[99] += + acadoWorkspace.evGx[1044]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1045]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1046]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1047]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1048]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1049]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1050]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1051]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1052]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1053]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1054]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1055]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[87];
acadoVariables.x[100] += + acadoWorkspace.evGx[1056]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1057]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1058]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1059]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1060]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1061]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1062]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1063]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1064]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1065]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1066]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1067]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[88];
acadoVariables.x[101] += + acadoWorkspace.evGx[1068]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1069]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1070]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1071]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1072]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1073]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1074]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1075]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1076]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1077]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1078]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1079]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[89];
acadoVariables.x[102] += + acadoWorkspace.evGx[1080]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1081]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1082]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1083]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1084]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1085]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1086]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1087]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1088]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1089]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1090]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1091]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[90];
acadoVariables.x[103] += + acadoWorkspace.evGx[1092]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1093]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1094]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1095]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1096]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1097]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1098]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1099]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1100]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1101]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1102]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1103]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[91];
acadoVariables.x[104] += + acadoWorkspace.evGx[1104]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1105]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1106]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1107]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1108]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1109]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1110]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1111]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1112]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1113]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1114]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1115]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[92];
acadoVariables.x[105] += + acadoWorkspace.evGx[1116]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1117]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1118]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1119]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1120]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1121]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1122]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1123]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1124]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1125]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1126]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1127]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[93];
acadoVariables.x[106] += + acadoWorkspace.evGx[1128]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1129]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1130]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1131]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1132]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1133]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1134]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1135]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1136]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1137]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1138]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1139]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[94];
acadoVariables.x[107] += + acadoWorkspace.evGx[1140]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1141]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1142]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1143]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1144]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1145]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1146]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1147]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1148]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1149]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1150]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1151]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[95];
acadoVariables.x[108] += + acadoWorkspace.evGx[1152]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1153]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1154]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1155]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1156]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1157]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1158]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1159]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1160]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1161]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1162]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1163]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[96];
acadoVariables.x[109] += + acadoWorkspace.evGx[1164]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1165]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1166]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1167]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1168]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1169]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1170]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1171]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1172]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1173]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1174]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1175]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[97];
acadoVariables.x[110] += + acadoWorkspace.evGx[1176]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1177]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1178]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1179]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1180]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1181]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1182]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1183]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1184]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1185]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1186]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1187]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[98];
acadoVariables.x[111] += + acadoWorkspace.evGx[1188]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1189]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1190]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1191]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1192]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1193]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1194]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1195]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1196]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1197]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1198]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1199]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[99];
acadoVariables.x[112] += + acadoWorkspace.evGx[1200]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1201]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1202]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1203]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1204]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1205]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1206]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1207]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1208]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1209]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1210]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1211]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[100];
acadoVariables.x[113] += + acadoWorkspace.evGx[1212]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1213]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1214]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1215]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1216]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1217]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1218]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1219]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1220]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1221]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1222]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1223]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[101];
acadoVariables.x[114] += + acadoWorkspace.evGx[1224]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1225]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1226]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1227]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1228]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1229]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1230]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1231]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1232]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1233]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1234]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1235]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[102];
acadoVariables.x[115] += + acadoWorkspace.evGx[1236]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1237]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1238]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1239]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1240]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1241]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1242]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1243]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1244]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1245]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1246]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1247]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[103];
acadoVariables.x[116] += + acadoWorkspace.evGx[1248]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1249]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1250]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1251]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1252]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1253]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1254]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1255]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1256]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1257]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1258]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1259]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[104];
acadoVariables.x[117] += + acadoWorkspace.evGx[1260]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1261]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1262]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1263]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1264]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1265]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1266]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1267]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1268]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1269]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1270]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1271]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[105];
acadoVariables.x[118] += + acadoWorkspace.evGx[1272]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1273]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1274]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1275]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1276]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1277]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1278]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1279]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1280]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1281]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1282]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1283]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[106];
acadoVariables.x[119] += + acadoWorkspace.evGx[1284]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1285]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1286]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1287]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1288]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1289]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1290]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1291]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1292]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1293]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1294]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1295]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[107];
acadoVariables.x[120] += + acadoWorkspace.evGx[1296]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1297]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1298]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1299]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1300]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1301]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1302]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1303]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1304]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1305]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1306]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1307]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[108];
acadoVariables.x[121] += + acadoWorkspace.evGx[1308]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1309]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1310]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1311]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1312]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1313]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1314]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1315]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1316]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1317]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1318]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1319]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[109];
acadoVariables.x[122] += + acadoWorkspace.evGx[1320]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1321]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1322]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1323]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1324]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1325]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1326]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1327]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1328]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1329]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1330]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1331]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[110];
acadoVariables.x[123] += + acadoWorkspace.evGx[1332]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1333]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1334]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1335]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1336]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1337]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1338]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1339]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1340]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1341]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1342]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1343]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[111];
acadoVariables.x[124] += + acadoWorkspace.evGx[1344]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1345]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1346]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1347]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1348]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1349]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1350]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1351]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1352]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1353]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1354]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1355]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[112];
acadoVariables.x[125] += + acadoWorkspace.evGx[1356]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1357]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1358]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1359]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1360]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1361]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1362]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1363]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1364]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1365]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1366]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1367]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[113];
acadoVariables.x[126] += + acadoWorkspace.evGx[1368]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1369]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1370]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1371]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1372]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1373]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1374]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1375]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1376]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1377]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1378]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1379]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[114];
acadoVariables.x[127] += + acadoWorkspace.evGx[1380]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1381]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1382]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1383]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1384]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1385]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1386]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1387]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1388]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1389]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1390]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1391]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[115];
acadoVariables.x[128] += + acadoWorkspace.evGx[1392]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1393]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1394]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1395]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1396]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1397]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1398]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1399]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1400]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1401]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1402]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1403]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[116];
acadoVariables.x[129] += + acadoWorkspace.evGx[1404]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1405]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1406]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1407]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1408]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1409]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1410]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1411]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1412]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1413]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1414]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1415]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[117];
acadoVariables.x[130] += + acadoWorkspace.evGx[1416]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1417]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1418]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1419]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1420]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1421]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1422]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1423]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1424]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1425]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1426]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1427]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[118];
acadoVariables.x[131] += + acadoWorkspace.evGx[1428]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1429]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1430]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1431]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1432]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1433]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1434]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1435]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1436]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1437]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1438]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1439]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[119];
acadoVariables.x[132] += + acadoWorkspace.evGx[1440]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1441]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1442]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1443]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1444]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1445]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1446]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1447]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1448]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1449]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1450]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1451]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[120];
acadoVariables.x[133] += + acadoWorkspace.evGx[1452]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1453]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1454]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1455]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1456]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1457]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1458]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1459]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1460]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1461]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1462]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1463]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[121];
acadoVariables.x[134] += + acadoWorkspace.evGx[1464]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1465]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1466]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1467]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1468]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1469]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1470]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1471]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1472]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1473]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1474]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1475]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[122];
acadoVariables.x[135] += + acadoWorkspace.evGx[1476]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1477]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1478]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1479]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1480]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1481]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1482]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1483]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1484]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1485]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1486]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1487]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[123];
acadoVariables.x[136] += + acadoWorkspace.evGx[1488]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1489]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1490]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1491]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1492]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1493]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1494]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1495]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1496]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1497]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1498]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1499]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[124];
acadoVariables.x[137] += + acadoWorkspace.evGx[1500]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1501]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1502]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1503]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1504]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1505]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1506]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1507]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1508]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1509]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1510]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1511]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[125];
acadoVariables.x[138] += + acadoWorkspace.evGx[1512]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1513]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1514]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1515]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1516]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1517]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1518]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1519]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1520]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1521]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1522]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1523]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[126];
acadoVariables.x[139] += + acadoWorkspace.evGx[1524]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1525]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1526]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1527]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1528]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1529]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1530]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1531]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1532]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1533]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1534]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1535]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[127];
acadoVariables.x[140] += + acadoWorkspace.evGx[1536]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1537]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1538]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1539]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1540]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1541]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1542]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1543]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1544]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1545]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1546]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1547]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[128];
acadoVariables.x[141] += + acadoWorkspace.evGx[1548]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1549]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1550]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1551]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1552]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1553]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1554]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1555]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1556]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1557]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1558]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1559]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[129];
acadoVariables.x[142] += + acadoWorkspace.evGx[1560]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1561]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1562]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1563]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1564]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1565]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1566]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1567]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1568]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1569]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1570]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1571]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[130];
acadoVariables.x[143] += + acadoWorkspace.evGx[1572]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1573]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1574]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1575]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1576]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1577]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1578]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1579]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1580]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1581]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1582]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1583]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[131];
acadoVariables.x[144] += + acadoWorkspace.evGx[1584]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1585]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1586]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1587]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1588]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1589]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1590]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1591]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1592]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1593]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1594]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1595]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[132];
acadoVariables.x[145] += + acadoWorkspace.evGx[1596]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1597]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1598]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1599]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1600]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1601]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1602]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1603]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1604]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1605]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1606]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1607]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[133];
acadoVariables.x[146] += + acadoWorkspace.evGx[1608]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1609]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1610]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1611]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1612]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1613]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1614]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1615]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1616]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1617]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1618]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1619]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[134];
acadoVariables.x[147] += + acadoWorkspace.evGx[1620]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1621]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1622]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1623]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1624]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1625]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1626]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1627]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1628]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1629]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1630]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1631]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[135];
acadoVariables.x[148] += + acadoWorkspace.evGx[1632]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1633]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1634]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1635]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1636]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1637]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1638]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1639]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1640]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1641]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1642]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1643]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[136];
acadoVariables.x[149] += + acadoWorkspace.evGx[1644]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1645]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1646]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1647]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1648]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1649]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1650]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1651]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1652]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1653]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1654]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1655]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[137];
acadoVariables.x[150] += + acadoWorkspace.evGx[1656]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1657]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1658]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1659]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1660]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1661]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1662]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1663]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1664]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1665]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1666]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1667]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[138];
acadoVariables.x[151] += + acadoWorkspace.evGx[1668]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1669]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1670]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1671]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1672]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1673]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1674]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1675]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1676]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1677]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1678]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1679]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[139];
acadoVariables.x[152] += + acadoWorkspace.evGx[1680]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1681]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1682]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1683]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1684]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1685]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1686]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1687]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1688]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1689]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1690]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1691]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[140];
acadoVariables.x[153] += + acadoWorkspace.evGx[1692]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1693]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1694]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1695]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1696]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1697]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1698]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1699]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1700]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1701]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1702]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1703]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[141];
acadoVariables.x[154] += + acadoWorkspace.evGx[1704]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1705]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1706]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1707]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1708]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1709]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1710]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1711]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1712]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1713]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1714]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1715]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[142];
acadoVariables.x[155] += + acadoWorkspace.evGx[1716]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1717]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1718]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1719]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1720]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1721]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1722]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1723]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1724]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1725]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1726]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1727]*acadoWorkspace.Dx0[11] + acadoWorkspace.d[143];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 48 ]), acadoWorkspace.x, &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 144 ]), acadoWorkspace.x, &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 288 ]), acadoWorkspace.x, &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 480 ]), acadoWorkspace.x, &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 720 ]), acadoWorkspace.x, &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1008 ]), acadoWorkspace.x, &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1344 ]), acadoWorkspace.x, &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1728 ]), acadoWorkspace.x, &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2160 ]), acadoWorkspace.x, &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2544 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2640 ]), acadoWorkspace.x, &(acadoVariables.x[ 132 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2688 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 132 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2736 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 132 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2784 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 132 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2832 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 132 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2880 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 132 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2928 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 132 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2976 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 132 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3024 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 132 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3072 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 132 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3120 ]), &(acadoWorkspace.x[ 40 ]), &(acadoVariables.x[ 132 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3168 ]), acadoWorkspace.x, &(acadoVariables.x[ 144 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3216 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 144 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3264 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 144 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3312 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 144 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3360 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 144 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3408 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 144 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3456 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 144 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3504 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 144 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3552 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 144 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3600 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 144 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3648 ]), &(acadoWorkspace.x[ 40 ]), &(acadoVariables.x[ 144 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3696 ]), &(acadoWorkspace.x[ 44 ]), &(acadoVariables.x[ 144 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 12; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 12];
acadoWorkspace.state[1] = acadoVariables.x[index * 12 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 12 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 12 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 12 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 12 + 5];
acadoWorkspace.state[6] = acadoVariables.x[index * 12 + 6];
acadoWorkspace.state[7] = acadoVariables.x[index * 12 + 7];
acadoWorkspace.state[8] = acadoVariables.x[index * 12 + 8];
acadoWorkspace.state[9] = acadoVariables.x[index * 12 + 9];
acadoWorkspace.state[10] = acadoVariables.x[index * 12 + 10];
acadoWorkspace.state[11] = acadoVariables.x[index * 12 + 11];
acadoWorkspace.state[204] = acadoVariables.u[index * 4];
acadoWorkspace.state[205] = acadoVariables.u[index * 4 + 1];
acadoWorkspace.state[206] = acadoVariables.u[index * 4 + 2];
acadoWorkspace.state[207] = acadoVariables.u[index * 4 + 3];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 12 + 12] = acadoWorkspace.state[0];
acadoVariables.x[index * 12 + 13] = acadoWorkspace.state[1];
acadoVariables.x[index * 12 + 14] = acadoWorkspace.state[2];
acadoVariables.x[index * 12 + 15] = acadoWorkspace.state[3];
acadoVariables.x[index * 12 + 16] = acadoWorkspace.state[4];
acadoVariables.x[index * 12 + 17] = acadoWorkspace.state[5];
acadoVariables.x[index * 12 + 18] = acadoWorkspace.state[6];
acadoVariables.x[index * 12 + 19] = acadoWorkspace.state[7];
acadoVariables.x[index * 12 + 20] = acadoWorkspace.state[8];
acadoVariables.x[index * 12 + 21] = acadoWorkspace.state[9];
acadoVariables.x[index * 12 + 22] = acadoWorkspace.state[10];
acadoVariables.x[index * 12 + 23] = acadoWorkspace.state[11];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 12; ++index)
{
acadoVariables.x[index * 12] = acadoVariables.x[index * 12 + 12];
acadoVariables.x[index * 12 + 1] = acadoVariables.x[index * 12 + 13];
acadoVariables.x[index * 12 + 2] = acadoVariables.x[index * 12 + 14];
acadoVariables.x[index * 12 + 3] = acadoVariables.x[index * 12 + 15];
acadoVariables.x[index * 12 + 4] = acadoVariables.x[index * 12 + 16];
acadoVariables.x[index * 12 + 5] = acadoVariables.x[index * 12 + 17];
acadoVariables.x[index * 12 + 6] = acadoVariables.x[index * 12 + 18];
acadoVariables.x[index * 12 + 7] = acadoVariables.x[index * 12 + 19];
acadoVariables.x[index * 12 + 8] = acadoVariables.x[index * 12 + 20];
acadoVariables.x[index * 12 + 9] = acadoVariables.x[index * 12 + 21];
acadoVariables.x[index * 12 + 10] = acadoVariables.x[index * 12 + 22];
acadoVariables.x[index * 12 + 11] = acadoVariables.x[index * 12 + 23];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[144] = xEnd[0];
acadoVariables.x[145] = xEnd[1];
acadoVariables.x[146] = xEnd[2];
acadoVariables.x[147] = xEnd[3];
acadoVariables.x[148] = xEnd[4];
acadoVariables.x[149] = xEnd[5];
acadoVariables.x[150] = xEnd[6];
acadoVariables.x[151] = xEnd[7];
acadoVariables.x[152] = xEnd[8];
acadoVariables.x[153] = xEnd[9];
acadoVariables.x[154] = xEnd[10];
acadoVariables.x[155] = xEnd[11];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[144];
acadoWorkspace.state[1] = acadoVariables.x[145];
acadoWorkspace.state[2] = acadoVariables.x[146];
acadoWorkspace.state[3] = acadoVariables.x[147];
acadoWorkspace.state[4] = acadoVariables.x[148];
acadoWorkspace.state[5] = acadoVariables.x[149];
acadoWorkspace.state[6] = acadoVariables.x[150];
acadoWorkspace.state[7] = acadoVariables.x[151];
acadoWorkspace.state[8] = acadoVariables.x[152];
acadoWorkspace.state[9] = acadoVariables.x[153];
acadoWorkspace.state[10] = acadoVariables.x[154];
acadoWorkspace.state[11] = acadoVariables.x[155];
if (uEnd != 0)
{
acadoWorkspace.state[204] = uEnd[0];
acadoWorkspace.state[205] = uEnd[1];
acadoWorkspace.state[206] = uEnd[2];
acadoWorkspace.state[207] = uEnd[3];
}
else
{
acadoWorkspace.state[204] = acadoVariables.u[44];
acadoWorkspace.state[205] = acadoVariables.u[45];
acadoWorkspace.state[206] = acadoVariables.u[46];
acadoWorkspace.state[207] = acadoVariables.u[47];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[144] = acadoWorkspace.state[0];
acadoVariables.x[145] = acadoWorkspace.state[1];
acadoVariables.x[146] = acadoWorkspace.state[2];
acadoVariables.x[147] = acadoWorkspace.state[3];
acadoVariables.x[148] = acadoWorkspace.state[4];
acadoVariables.x[149] = acadoWorkspace.state[5];
acadoVariables.x[150] = acadoWorkspace.state[6];
acadoVariables.x[151] = acadoWorkspace.state[7];
acadoVariables.x[152] = acadoWorkspace.state[8];
acadoVariables.x[153] = acadoWorkspace.state[9];
acadoVariables.x[154] = acadoWorkspace.state[10];
acadoVariables.x[155] = acadoWorkspace.state[11];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 11; ++index)
{
acadoVariables.u[index * 4] = acadoVariables.u[index * 4 + 4];
acadoVariables.u[index * 4 + 1] = acadoVariables.u[index * 4 + 5];
acadoVariables.u[index * 4 + 2] = acadoVariables.u[index * 4 + 6];
acadoVariables.u[index * 4 + 3] = acadoVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
acadoVariables.u[44] = uEnd[0];
acadoVariables.u[45] = uEnd[1];
acadoVariables.u[46] = uEnd[2];
acadoVariables.u[47] = uEnd[3];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47];
kkt = fabs( kkt );
for (index = 0; index < 48; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 8 */
real_t tmpDy[ 8 ];

/** Row vector of size: 4 */
real_t tmpDyN[ 4 ];

for (lRun1 = 0; lRun1 < 12; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 12];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 12 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 12 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 12 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 12 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 12 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 12 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 12 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 12 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[lRun1 * 12 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.x[lRun1 * 12 + 10];
acadoWorkspace.objValueIn[11] = acadoVariables.x[lRun1 * 12 + 11];
acadoWorkspace.objValueIn[12] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.objValueIn[13] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.u[lRun1 * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 8] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 8];
acadoWorkspace.Dy[lRun1 * 8 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 8 + 1];
acadoWorkspace.Dy[lRun1 * 8 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 8 + 2];
acadoWorkspace.Dy[lRun1 * 8 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 8 + 3];
acadoWorkspace.Dy[lRun1 * 8 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 8 + 4];
acadoWorkspace.Dy[lRun1 * 8 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 8 + 5];
acadoWorkspace.Dy[lRun1 * 8 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 8 + 6];
acadoWorkspace.Dy[lRun1 * 8 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 8 + 7];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[144];
acadoWorkspace.objValueIn[1] = acadoVariables.x[145];
acadoWorkspace.objValueIn[2] = acadoVariables.x[146];
acadoWorkspace.objValueIn[3] = acadoVariables.x[147];
acadoWorkspace.objValueIn[4] = acadoVariables.x[148];
acadoWorkspace.objValueIn[5] = acadoVariables.x[149];
acadoWorkspace.objValueIn[6] = acadoVariables.x[150];
acadoWorkspace.objValueIn[7] = acadoVariables.x[151];
acadoWorkspace.objValueIn[8] = acadoVariables.x[152];
acadoWorkspace.objValueIn[9] = acadoVariables.x[153];
acadoWorkspace.objValueIn[10] = acadoVariables.x[154];
acadoWorkspace.objValueIn[11] = acadoVariables.x[155];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 12; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 8]*(real_t)2.5500000000000000e+01;
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 8 + 1]*(real_t)1.4750000000000000e+02;
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 8 + 2]*(real_t)1.4750000000000000e+02;
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 8 + 3]*(real_t)1.4750000000000000e+02;
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 8 + 4]*(real_t)2.5000000000000000e-01;
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 8 + 5]*(real_t)5.7500000000000000e+00;
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 8 + 6]*(real_t)5.7500000000000000e+00;
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 8 + 7]*(real_t)5.7500000000000000e+00;
objVal += + acadoWorkspace.Dy[lRun1 * 8]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 8 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 8 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 8 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 8 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 8 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 8 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 8 + 7]*tmpDy[7];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)2.5000000000000001e-02;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)2.5000000000000000e-01;
tmpDyN[2] = + acadoWorkspace.DyN[2]*(real_t)2.5000000000000000e-01;
tmpDyN[3] = + acadoWorkspace.DyN[3]*(real_t)2.5000000000000000e-01;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3];

objVal *= 0.5;
return objVal;
}

