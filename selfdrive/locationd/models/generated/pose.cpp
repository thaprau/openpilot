#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4220870942759577714) {
   out_4220870942759577714[0] = delta_x[0] + nom_x[0];
   out_4220870942759577714[1] = delta_x[1] + nom_x[1];
   out_4220870942759577714[2] = delta_x[2] + nom_x[2];
   out_4220870942759577714[3] = delta_x[3] + nom_x[3];
   out_4220870942759577714[4] = delta_x[4] + nom_x[4];
   out_4220870942759577714[5] = delta_x[5] + nom_x[5];
   out_4220870942759577714[6] = delta_x[6] + nom_x[6];
   out_4220870942759577714[7] = delta_x[7] + nom_x[7];
   out_4220870942759577714[8] = delta_x[8] + nom_x[8];
   out_4220870942759577714[9] = delta_x[9] + nom_x[9];
   out_4220870942759577714[10] = delta_x[10] + nom_x[10];
   out_4220870942759577714[11] = delta_x[11] + nom_x[11];
   out_4220870942759577714[12] = delta_x[12] + nom_x[12];
   out_4220870942759577714[13] = delta_x[13] + nom_x[13];
   out_4220870942759577714[14] = delta_x[14] + nom_x[14];
   out_4220870942759577714[15] = delta_x[15] + nom_x[15];
   out_4220870942759577714[16] = delta_x[16] + nom_x[16];
   out_4220870942759577714[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6691487759945560833) {
   out_6691487759945560833[0] = -nom_x[0] + true_x[0];
   out_6691487759945560833[1] = -nom_x[1] + true_x[1];
   out_6691487759945560833[2] = -nom_x[2] + true_x[2];
   out_6691487759945560833[3] = -nom_x[3] + true_x[3];
   out_6691487759945560833[4] = -nom_x[4] + true_x[4];
   out_6691487759945560833[5] = -nom_x[5] + true_x[5];
   out_6691487759945560833[6] = -nom_x[6] + true_x[6];
   out_6691487759945560833[7] = -nom_x[7] + true_x[7];
   out_6691487759945560833[8] = -nom_x[8] + true_x[8];
   out_6691487759945560833[9] = -nom_x[9] + true_x[9];
   out_6691487759945560833[10] = -nom_x[10] + true_x[10];
   out_6691487759945560833[11] = -nom_x[11] + true_x[11];
   out_6691487759945560833[12] = -nom_x[12] + true_x[12];
   out_6691487759945560833[13] = -nom_x[13] + true_x[13];
   out_6691487759945560833[14] = -nom_x[14] + true_x[14];
   out_6691487759945560833[15] = -nom_x[15] + true_x[15];
   out_6691487759945560833[16] = -nom_x[16] + true_x[16];
   out_6691487759945560833[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_5674745270205462321) {
   out_5674745270205462321[0] = 1.0;
   out_5674745270205462321[1] = 0.0;
   out_5674745270205462321[2] = 0.0;
   out_5674745270205462321[3] = 0.0;
   out_5674745270205462321[4] = 0.0;
   out_5674745270205462321[5] = 0.0;
   out_5674745270205462321[6] = 0.0;
   out_5674745270205462321[7] = 0.0;
   out_5674745270205462321[8] = 0.0;
   out_5674745270205462321[9] = 0.0;
   out_5674745270205462321[10] = 0.0;
   out_5674745270205462321[11] = 0.0;
   out_5674745270205462321[12] = 0.0;
   out_5674745270205462321[13] = 0.0;
   out_5674745270205462321[14] = 0.0;
   out_5674745270205462321[15] = 0.0;
   out_5674745270205462321[16] = 0.0;
   out_5674745270205462321[17] = 0.0;
   out_5674745270205462321[18] = 0.0;
   out_5674745270205462321[19] = 1.0;
   out_5674745270205462321[20] = 0.0;
   out_5674745270205462321[21] = 0.0;
   out_5674745270205462321[22] = 0.0;
   out_5674745270205462321[23] = 0.0;
   out_5674745270205462321[24] = 0.0;
   out_5674745270205462321[25] = 0.0;
   out_5674745270205462321[26] = 0.0;
   out_5674745270205462321[27] = 0.0;
   out_5674745270205462321[28] = 0.0;
   out_5674745270205462321[29] = 0.0;
   out_5674745270205462321[30] = 0.0;
   out_5674745270205462321[31] = 0.0;
   out_5674745270205462321[32] = 0.0;
   out_5674745270205462321[33] = 0.0;
   out_5674745270205462321[34] = 0.0;
   out_5674745270205462321[35] = 0.0;
   out_5674745270205462321[36] = 0.0;
   out_5674745270205462321[37] = 0.0;
   out_5674745270205462321[38] = 1.0;
   out_5674745270205462321[39] = 0.0;
   out_5674745270205462321[40] = 0.0;
   out_5674745270205462321[41] = 0.0;
   out_5674745270205462321[42] = 0.0;
   out_5674745270205462321[43] = 0.0;
   out_5674745270205462321[44] = 0.0;
   out_5674745270205462321[45] = 0.0;
   out_5674745270205462321[46] = 0.0;
   out_5674745270205462321[47] = 0.0;
   out_5674745270205462321[48] = 0.0;
   out_5674745270205462321[49] = 0.0;
   out_5674745270205462321[50] = 0.0;
   out_5674745270205462321[51] = 0.0;
   out_5674745270205462321[52] = 0.0;
   out_5674745270205462321[53] = 0.0;
   out_5674745270205462321[54] = 0.0;
   out_5674745270205462321[55] = 0.0;
   out_5674745270205462321[56] = 0.0;
   out_5674745270205462321[57] = 1.0;
   out_5674745270205462321[58] = 0.0;
   out_5674745270205462321[59] = 0.0;
   out_5674745270205462321[60] = 0.0;
   out_5674745270205462321[61] = 0.0;
   out_5674745270205462321[62] = 0.0;
   out_5674745270205462321[63] = 0.0;
   out_5674745270205462321[64] = 0.0;
   out_5674745270205462321[65] = 0.0;
   out_5674745270205462321[66] = 0.0;
   out_5674745270205462321[67] = 0.0;
   out_5674745270205462321[68] = 0.0;
   out_5674745270205462321[69] = 0.0;
   out_5674745270205462321[70] = 0.0;
   out_5674745270205462321[71] = 0.0;
   out_5674745270205462321[72] = 0.0;
   out_5674745270205462321[73] = 0.0;
   out_5674745270205462321[74] = 0.0;
   out_5674745270205462321[75] = 0.0;
   out_5674745270205462321[76] = 1.0;
   out_5674745270205462321[77] = 0.0;
   out_5674745270205462321[78] = 0.0;
   out_5674745270205462321[79] = 0.0;
   out_5674745270205462321[80] = 0.0;
   out_5674745270205462321[81] = 0.0;
   out_5674745270205462321[82] = 0.0;
   out_5674745270205462321[83] = 0.0;
   out_5674745270205462321[84] = 0.0;
   out_5674745270205462321[85] = 0.0;
   out_5674745270205462321[86] = 0.0;
   out_5674745270205462321[87] = 0.0;
   out_5674745270205462321[88] = 0.0;
   out_5674745270205462321[89] = 0.0;
   out_5674745270205462321[90] = 0.0;
   out_5674745270205462321[91] = 0.0;
   out_5674745270205462321[92] = 0.0;
   out_5674745270205462321[93] = 0.0;
   out_5674745270205462321[94] = 0.0;
   out_5674745270205462321[95] = 1.0;
   out_5674745270205462321[96] = 0.0;
   out_5674745270205462321[97] = 0.0;
   out_5674745270205462321[98] = 0.0;
   out_5674745270205462321[99] = 0.0;
   out_5674745270205462321[100] = 0.0;
   out_5674745270205462321[101] = 0.0;
   out_5674745270205462321[102] = 0.0;
   out_5674745270205462321[103] = 0.0;
   out_5674745270205462321[104] = 0.0;
   out_5674745270205462321[105] = 0.0;
   out_5674745270205462321[106] = 0.0;
   out_5674745270205462321[107] = 0.0;
   out_5674745270205462321[108] = 0.0;
   out_5674745270205462321[109] = 0.0;
   out_5674745270205462321[110] = 0.0;
   out_5674745270205462321[111] = 0.0;
   out_5674745270205462321[112] = 0.0;
   out_5674745270205462321[113] = 0.0;
   out_5674745270205462321[114] = 1.0;
   out_5674745270205462321[115] = 0.0;
   out_5674745270205462321[116] = 0.0;
   out_5674745270205462321[117] = 0.0;
   out_5674745270205462321[118] = 0.0;
   out_5674745270205462321[119] = 0.0;
   out_5674745270205462321[120] = 0.0;
   out_5674745270205462321[121] = 0.0;
   out_5674745270205462321[122] = 0.0;
   out_5674745270205462321[123] = 0.0;
   out_5674745270205462321[124] = 0.0;
   out_5674745270205462321[125] = 0.0;
   out_5674745270205462321[126] = 0.0;
   out_5674745270205462321[127] = 0.0;
   out_5674745270205462321[128] = 0.0;
   out_5674745270205462321[129] = 0.0;
   out_5674745270205462321[130] = 0.0;
   out_5674745270205462321[131] = 0.0;
   out_5674745270205462321[132] = 0.0;
   out_5674745270205462321[133] = 1.0;
   out_5674745270205462321[134] = 0.0;
   out_5674745270205462321[135] = 0.0;
   out_5674745270205462321[136] = 0.0;
   out_5674745270205462321[137] = 0.0;
   out_5674745270205462321[138] = 0.0;
   out_5674745270205462321[139] = 0.0;
   out_5674745270205462321[140] = 0.0;
   out_5674745270205462321[141] = 0.0;
   out_5674745270205462321[142] = 0.0;
   out_5674745270205462321[143] = 0.0;
   out_5674745270205462321[144] = 0.0;
   out_5674745270205462321[145] = 0.0;
   out_5674745270205462321[146] = 0.0;
   out_5674745270205462321[147] = 0.0;
   out_5674745270205462321[148] = 0.0;
   out_5674745270205462321[149] = 0.0;
   out_5674745270205462321[150] = 0.0;
   out_5674745270205462321[151] = 0.0;
   out_5674745270205462321[152] = 1.0;
   out_5674745270205462321[153] = 0.0;
   out_5674745270205462321[154] = 0.0;
   out_5674745270205462321[155] = 0.0;
   out_5674745270205462321[156] = 0.0;
   out_5674745270205462321[157] = 0.0;
   out_5674745270205462321[158] = 0.0;
   out_5674745270205462321[159] = 0.0;
   out_5674745270205462321[160] = 0.0;
   out_5674745270205462321[161] = 0.0;
   out_5674745270205462321[162] = 0.0;
   out_5674745270205462321[163] = 0.0;
   out_5674745270205462321[164] = 0.0;
   out_5674745270205462321[165] = 0.0;
   out_5674745270205462321[166] = 0.0;
   out_5674745270205462321[167] = 0.0;
   out_5674745270205462321[168] = 0.0;
   out_5674745270205462321[169] = 0.0;
   out_5674745270205462321[170] = 0.0;
   out_5674745270205462321[171] = 1.0;
   out_5674745270205462321[172] = 0.0;
   out_5674745270205462321[173] = 0.0;
   out_5674745270205462321[174] = 0.0;
   out_5674745270205462321[175] = 0.0;
   out_5674745270205462321[176] = 0.0;
   out_5674745270205462321[177] = 0.0;
   out_5674745270205462321[178] = 0.0;
   out_5674745270205462321[179] = 0.0;
   out_5674745270205462321[180] = 0.0;
   out_5674745270205462321[181] = 0.0;
   out_5674745270205462321[182] = 0.0;
   out_5674745270205462321[183] = 0.0;
   out_5674745270205462321[184] = 0.0;
   out_5674745270205462321[185] = 0.0;
   out_5674745270205462321[186] = 0.0;
   out_5674745270205462321[187] = 0.0;
   out_5674745270205462321[188] = 0.0;
   out_5674745270205462321[189] = 0.0;
   out_5674745270205462321[190] = 1.0;
   out_5674745270205462321[191] = 0.0;
   out_5674745270205462321[192] = 0.0;
   out_5674745270205462321[193] = 0.0;
   out_5674745270205462321[194] = 0.0;
   out_5674745270205462321[195] = 0.0;
   out_5674745270205462321[196] = 0.0;
   out_5674745270205462321[197] = 0.0;
   out_5674745270205462321[198] = 0.0;
   out_5674745270205462321[199] = 0.0;
   out_5674745270205462321[200] = 0.0;
   out_5674745270205462321[201] = 0.0;
   out_5674745270205462321[202] = 0.0;
   out_5674745270205462321[203] = 0.0;
   out_5674745270205462321[204] = 0.0;
   out_5674745270205462321[205] = 0.0;
   out_5674745270205462321[206] = 0.0;
   out_5674745270205462321[207] = 0.0;
   out_5674745270205462321[208] = 0.0;
   out_5674745270205462321[209] = 1.0;
   out_5674745270205462321[210] = 0.0;
   out_5674745270205462321[211] = 0.0;
   out_5674745270205462321[212] = 0.0;
   out_5674745270205462321[213] = 0.0;
   out_5674745270205462321[214] = 0.0;
   out_5674745270205462321[215] = 0.0;
   out_5674745270205462321[216] = 0.0;
   out_5674745270205462321[217] = 0.0;
   out_5674745270205462321[218] = 0.0;
   out_5674745270205462321[219] = 0.0;
   out_5674745270205462321[220] = 0.0;
   out_5674745270205462321[221] = 0.0;
   out_5674745270205462321[222] = 0.0;
   out_5674745270205462321[223] = 0.0;
   out_5674745270205462321[224] = 0.0;
   out_5674745270205462321[225] = 0.0;
   out_5674745270205462321[226] = 0.0;
   out_5674745270205462321[227] = 0.0;
   out_5674745270205462321[228] = 1.0;
   out_5674745270205462321[229] = 0.0;
   out_5674745270205462321[230] = 0.0;
   out_5674745270205462321[231] = 0.0;
   out_5674745270205462321[232] = 0.0;
   out_5674745270205462321[233] = 0.0;
   out_5674745270205462321[234] = 0.0;
   out_5674745270205462321[235] = 0.0;
   out_5674745270205462321[236] = 0.0;
   out_5674745270205462321[237] = 0.0;
   out_5674745270205462321[238] = 0.0;
   out_5674745270205462321[239] = 0.0;
   out_5674745270205462321[240] = 0.0;
   out_5674745270205462321[241] = 0.0;
   out_5674745270205462321[242] = 0.0;
   out_5674745270205462321[243] = 0.0;
   out_5674745270205462321[244] = 0.0;
   out_5674745270205462321[245] = 0.0;
   out_5674745270205462321[246] = 0.0;
   out_5674745270205462321[247] = 1.0;
   out_5674745270205462321[248] = 0.0;
   out_5674745270205462321[249] = 0.0;
   out_5674745270205462321[250] = 0.0;
   out_5674745270205462321[251] = 0.0;
   out_5674745270205462321[252] = 0.0;
   out_5674745270205462321[253] = 0.0;
   out_5674745270205462321[254] = 0.0;
   out_5674745270205462321[255] = 0.0;
   out_5674745270205462321[256] = 0.0;
   out_5674745270205462321[257] = 0.0;
   out_5674745270205462321[258] = 0.0;
   out_5674745270205462321[259] = 0.0;
   out_5674745270205462321[260] = 0.0;
   out_5674745270205462321[261] = 0.0;
   out_5674745270205462321[262] = 0.0;
   out_5674745270205462321[263] = 0.0;
   out_5674745270205462321[264] = 0.0;
   out_5674745270205462321[265] = 0.0;
   out_5674745270205462321[266] = 1.0;
   out_5674745270205462321[267] = 0.0;
   out_5674745270205462321[268] = 0.0;
   out_5674745270205462321[269] = 0.0;
   out_5674745270205462321[270] = 0.0;
   out_5674745270205462321[271] = 0.0;
   out_5674745270205462321[272] = 0.0;
   out_5674745270205462321[273] = 0.0;
   out_5674745270205462321[274] = 0.0;
   out_5674745270205462321[275] = 0.0;
   out_5674745270205462321[276] = 0.0;
   out_5674745270205462321[277] = 0.0;
   out_5674745270205462321[278] = 0.0;
   out_5674745270205462321[279] = 0.0;
   out_5674745270205462321[280] = 0.0;
   out_5674745270205462321[281] = 0.0;
   out_5674745270205462321[282] = 0.0;
   out_5674745270205462321[283] = 0.0;
   out_5674745270205462321[284] = 0.0;
   out_5674745270205462321[285] = 1.0;
   out_5674745270205462321[286] = 0.0;
   out_5674745270205462321[287] = 0.0;
   out_5674745270205462321[288] = 0.0;
   out_5674745270205462321[289] = 0.0;
   out_5674745270205462321[290] = 0.0;
   out_5674745270205462321[291] = 0.0;
   out_5674745270205462321[292] = 0.0;
   out_5674745270205462321[293] = 0.0;
   out_5674745270205462321[294] = 0.0;
   out_5674745270205462321[295] = 0.0;
   out_5674745270205462321[296] = 0.0;
   out_5674745270205462321[297] = 0.0;
   out_5674745270205462321[298] = 0.0;
   out_5674745270205462321[299] = 0.0;
   out_5674745270205462321[300] = 0.0;
   out_5674745270205462321[301] = 0.0;
   out_5674745270205462321[302] = 0.0;
   out_5674745270205462321[303] = 0.0;
   out_5674745270205462321[304] = 1.0;
   out_5674745270205462321[305] = 0.0;
   out_5674745270205462321[306] = 0.0;
   out_5674745270205462321[307] = 0.0;
   out_5674745270205462321[308] = 0.0;
   out_5674745270205462321[309] = 0.0;
   out_5674745270205462321[310] = 0.0;
   out_5674745270205462321[311] = 0.0;
   out_5674745270205462321[312] = 0.0;
   out_5674745270205462321[313] = 0.0;
   out_5674745270205462321[314] = 0.0;
   out_5674745270205462321[315] = 0.0;
   out_5674745270205462321[316] = 0.0;
   out_5674745270205462321[317] = 0.0;
   out_5674745270205462321[318] = 0.0;
   out_5674745270205462321[319] = 0.0;
   out_5674745270205462321[320] = 0.0;
   out_5674745270205462321[321] = 0.0;
   out_5674745270205462321[322] = 0.0;
   out_5674745270205462321[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_8950409172442745226) {
   out_8950409172442745226[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_8950409172442745226[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_8950409172442745226[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_8950409172442745226[3] = dt*state[12] + state[3];
   out_8950409172442745226[4] = dt*state[13] + state[4];
   out_8950409172442745226[5] = dt*state[14] + state[5];
   out_8950409172442745226[6] = state[6];
   out_8950409172442745226[7] = state[7];
   out_8950409172442745226[8] = state[8];
   out_8950409172442745226[9] = state[9];
   out_8950409172442745226[10] = state[10];
   out_8950409172442745226[11] = state[11];
   out_8950409172442745226[12] = state[12];
   out_8950409172442745226[13] = state[13];
   out_8950409172442745226[14] = state[14];
   out_8950409172442745226[15] = state[15];
   out_8950409172442745226[16] = state[16];
   out_8950409172442745226[17] = state[17];
}
void F_fun(double *state, double dt, double *out_7351665863096111010) {
   out_7351665863096111010[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7351665863096111010[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7351665863096111010[2] = 0;
   out_7351665863096111010[3] = 0;
   out_7351665863096111010[4] = 0;
   out_7351665863096111010[5] = 0;
   out_7351665863096111010[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7351665863096111010[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7351665863096111010[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7351665863096111010[9] = 0;
   out_7351665863096111010[10] = 0;
   out_7351665863096111010[11] = 0;
   out_7351665863096111010[12] = 0;
   out_7351665863096111010[13] = 0;
   out_7351665863096111010[14] = 0;
   out_7351665863096111010[15] = 0;
   out_7351665863096111010[16] = 0;
   out_7351665863096111010[17] = 0;
   out_7351665863096111010[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7351665863096111010[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7351665863096111010[20] = 0;
   out_7351665863096111010[21] = 0;
   out_7351665863096111010[22] = 0;
   out_7351665863096111010[23] = 0;
   out_7351665863096111010[24] = 0;
   out_7351665863096111010[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7351665863096111010[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7351665863096111010[27] = 0;
   out_7351665863096111010[28] = 0;
   out_7351665863096111010[29] = 0;
   out_7351665863096111010[30] = 0;
   out_7351665863096111010[31] = 0;
   out_7351665863096111010[32] = 0;
   out_7351665863096111010[33] = 0;
   out_7351665863096111010[34] = 0;
   out_7351665863096111010[35] = 0;
   out_7351665863096111010[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7351665863096111010[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7351665863096111010[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7351665863096111010[39] = 0;
   out_7351665863096111010[40] = 0;
   out_7351665863096111010[41] = 0;
   out_7351665863096111010[42] = 0;
   out_7351665863096111010[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7351665863096111010[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7351665863096111010[45] = 0;
   out_7351665863096111010[46] = 0;
   out_7351665863096111010[47] = 0;
   out_7351665863096111010[48] = 0;
   out_7351665863096111010[49] = 0;
   out_7351665863096111010[50] = 0;
   out_7351665863096111010[51] = 0;
   out_7351665863096111010[52] = 0;
   out_7351665863096111010[53] = 0;
   out_7351665863096111010[54] = 0;
   out_7351665863096111010[55] = 0;
   out_7351665863096111010[56] = 0;
   out_7351665863096111010[57] = 1;
   out_7351665863096111010[58] = 0;
   out_7351665863096111010[59] = 0;
   out_7351665863096111010[60] = 0;
   out_7351665863096111010[61] = 0;
   out_7351665863096111010[62] = 0;
   out_7351665863096111010[63] = 0;
   out_7351665863096111010[64] = 0;
   out_7351665863096111010[65] = 0;
   out_7351665863096111010[66] = dt;
   out_7351665863096111010[67] = 0;
   out_7351665863096111010[68] = 0;
   out_7351665863096111010[69] = 0;
   out_7351665863096111010[70] = 0;
   out_7351665863096111010[71] = 0;
   out_7351665863096111010[72] = 0;
   out_7351665863096111010[73] = 0;
   out_7351665863096111010[74] = 0;
   out_7351665863096111010[75] = 0;
   out_7351665863096111010[76] = 1;
   out_7351665863096111010[77] = 0;
   out_7351665863096111010[78] = 0;
   out_7351665863096111010[79] = 0;
   out_7351665863096111010[80] = 0;
   out_7351665863096111010[81] = 0;
   out_7351665863096111010[82] = 0;
   out_7351665863096111010[83] = 0;
   out_7351665863096111010[84] = 0;
   out_7351665863096111010[85] = dt;
   out_7351665863096111010[86] = 0;
   out_7351665863096111010[87] = 0;
   out_7351665863096111010[88] = 0;
   out_7351665863096111010[89] = 0;
   out_7351665863096111010[90] = 0;
   out_7351665863096111010[91] = 0;
   out_7351665863096111010[92] = 0;
   out_7351665863096111010[93] = 0;
   out_7351665863096111010[94] = 0;
   out_7351665863096111010[95] = 1;
   out_7351665863096111010[96] = 0;
   out_7351665863096111010[97] = 0;
   out_7351665863096111010[98] = 0;
   out_7351665863096111010[99] = 0;
   out_7351665863096111010[100] = 0;
   out_7351665863096111010[101] = 0;
   out_7351665863096111010[102] = 0;
   out_7351665863096111010[103] = 0;
   out_7351665863096111010[104] = dt;
   out_7351665863096111010[105] = 0;
   out_7351665863096111010[106] = 0;
   out_7351665863096111010[107] = 0;
   out_7351665863096111010[108] = 0;
   out_7351665863096111010[109] = 0;
   out_7351665863096111010[110] = 0;
   out_7351665863096111010[111] = 0;
   out_7351665863096111010[112] = 0;
   out_7351665863096111010[113] = 0;
   out_7351665863096111010[114] = 1;
   out_7351665863096111010[115] = 0;
   out_7351665863096111010[116] = 0;
   out_7351665863096111010[117] = 0;
   out_7351665863096111010[118] = 0;
   out_7351665863096111010[119] = 0;
   out_7351665863096111010[120] = 0;
   out_7351665863096111010[121] = 0;
   out_7351665863096111010[122] = 0;
   out_7351665863096111010[123] = 0;
   out_7351665863096111010[124] = 0;
   out_7351665863096111010[125] = 0;
   out_7351665863096111010[126] = 0;
   out_7351665863096111010[127] = 0;
   out_7351665863096111010[128] = 0;
   out_7351665863096111010[129] = 0;
   out_7351665863096111010[130] = 0;
   out_7351665863096111010[131] = 0;
   out_7351665863096111010[132] = 0;
   out_7351665863096111010[133] = 1;
   out_7351665863096111010[134] = 0;
   out_7351665863096111010[135] = 0;
   out_7351665863096111010[136] = 0;
   out_7351665863096111010[137] = 0;
   out_7351665863096111010[138] = 0;
   out_7351665863096111010[139] = 0;
   out_7351665863096111010[140] = 0;
   out_7351665863096111010[141] = 0;
   out_7351665863096111010[142] = 0;
   out_7351665863096111010[143] = 0;
   out_7351665863096111010[144] = 0;
   out_7351665863096111010[145] = 0;
   out_7351665863096111010[146] = 0;
   out_7351665863096111010[147] = 0;
   out_7351665863096111010[148] = 0;
   out_7351665863096111010[149] = 0;
   out_7351665863096111010[150] = 0;
   out_7351665863096111010[151] = 0;
   out_7351665863096111010[152] = 1;
   out_7351665863096111010[153] = 0;
   out_7351665863096111010[154] = 0;
   out_7351665863096111010[155] = 0;
   out_7351665863096111010[156] = 0;
   out_7351665863096111010[157] = 0;
   out_7351665863096111010[158] = 0;
   out_7351665863096111010[159] = 0;
   out_7351665863096111010[160] = 0;
   out_7351665863096111010[161] = 0;
   out_7351665863096111010[162] = 0;
   out_7351665863096111010[163] = 0;
   out_7351665863096111010[164] = 0;
   out_7351665863096111010[165] = 0;
   out_7351665863096111010[166] = 0;
   out_7351665863096111010[167] = 0;
   out_7351665863096111010[168] = 0;
   out_7351665863096111010[169] = 0;
   out_7351665863096111010[170] = 0;
   out_7351665863096111010[171] = 1;
   out_7351665863096111010[172] = 0;
   out_7351665863096111010[173] = 0;
   out_7351665863096111010[174] = 0;
   out_7351665863096111010[175] = 0;
   out_7351665863096111010[176] = 0;
   out_7351665863096111010[177] = 0;
   out_7351665863096111010[178] = 0;
   out_7351665863096111010[179] = 0;
   out_7351665863096111010[180] = 0;
   out_7351665863096111010[181] = 0;
   out_7351665863096111010[182] = 0;
   out_7351665863096111010[183] = 0;
   out_7351665863096111010[184] = 0;
   out_7351665863096111010[185] = 0;
   out_7351665863096111010[186] = 0;
   out_7351665863096111010[187] = 0;
   out_7351665863096111010[188] = 0;
   out_7351665863096111010[189] = 0;
   out_7351665863096111010[190] = 1;
   out_7351665863096111010[191] = 0;
   out_7351665863096111010[192] = 0;
   out_7351665863096111010[193] = 0;
   out_7351665863096111010[194] = 0;
   out_7351665863096111010[195] = 0;
   out_7351665863096111010[196] = 0;
   out_7351665863096111010[197] = 0;
   out_7351665863096111010[198] = 0;
   out_7351665863096111010[199] = 0;
   out_7351665863096111010[200] = 0;
   out_7351665863096111010[201] = 0;
   out_7351665863096111010[202] = 0;
   out_7351665863096111010[203] = 0;
   out_7351665863096111010[204] = 0;
   out_7351665863096111010[205] = 0;
   out_7351665863096111010[206] = 0;
   out_7351665863096111010[207] = 0;
   out_7351665863096111010[208] = 0;
   out_7351665863096111010[209] = 1;
   out_7351665863096111010[210] = 0;
   out_7351665863096111010[211] = 0;
   out_7351665863096111010[212] = 0;
   out_7351665863096111010[213] = 0;
   out_7351665863096111010[214] = 0;
   out_7351665863096111010[215] = 0;
   out_7351665863096111010[216] = 0;
   out_7351665863096111010[217] = 0;
   out_7351665863096111010[218] = 0;
   out_7351665863096111010[219] = 0;
   out_7351665863096111010[220] = 0;
   out_7351665863096111010[221] = 0;
   out_7351665863096111010[222] = 0;
   out_7351665863096111010[223] = 0;
   out_7351665863096111010[224] = 0;
   out_7351665863096111010[225] = 0;
   out_7351665863096111010[226] = 0;
   out_7351665863096111010[227] = 0;
   out_7351665863096111010[228] = 1;
   out_7351665863096111010[229] = 0;
   out_7351665863096111010[230] = 0;
   out_7351665863096111010[231] = 0;
   out_7351665863096111010[232] = 0;
   out_7351665863096111010[233] = 0;
   out_7351665863096111010[234] = 0;
   out_7351665863096111010[235] = 0;
   out_7351665863096111010[236] = 0;
   out_7351665863096111010[237] = 0;
   out_7351665863096111010[238] = 0;
   out_7351665863096111010[239] = 0;
   out_7351665863096111010[240] = 0;
   out_7351665863096111010[241] = 0;
   out_7351665863096111010[242] = 0;
   out_7351665863096111010[243] = 0;
   out_7351665863096111010[244] = 0;
   out_7351665863096111010[245] = 0;
   out_7351665863096111010[246] = 0;
   out_7351665863096111010[247] = 1;
   out_7351665863096111010[248] = 0;
   out_7351665863096111010[249] = 0;
   out_7351665863096111010[250] = 0;
   out_7351665863096111010[251] = 0;
   out_7351665863096111010[252] = 0;
   out_7351665863096111010[253] = 0;
   out_7351665863096111010[254] = 0;
   out_7351665863096111010[255] = 0;
   out_7351665863096111010[256] = 0;
   out_7351665863096111010[257] = 0;
   out_7351665863096111010[258] = 0;
   out_7351665863096111010[259] = 0;
   out_7351665863096111010[260] = 0;
   out_7351665863096111010[261] = 0;
   out_7351665863096111010[262] = 0;
   out_7351665863096111010[263] = 0;
   out_7351665863096111010[264] = 0;
   out_7351665863096111010[265] = 0;
   out_7351665863096111010[266] = 1;
   out_7351665863096111010[267] = 0;
   out_7351665863096111010[268] = 0;
   out_7351665863096111010[269] = 0;
   out_7351665863096111010[270] = 0;
   out_7351665863096111010[271] = 0;
   out_7351665863096111010[272] = 0;
   out_7351665863096111010[273] = 0;
   out_7351665863096111010[274] = 0;
   out_7351665863096111010[275] = 0;
   out_7351665863096111010[276] = 0;
   out_7351665863096111010[277] = 0;
   out_7351665863096111010[278] = 0;
   out_7351665863096111010[279] = 0;
   out_7351665863096111010[280] = 0;
   out_7351665863096111010[281] = 0;
   out_7351665863096111010[282] = 0;
   out_7351665863096111010[283] = 0;
   out_7351665863096111010[284] = 0;
   out_7351665863096111010[285] = 1;
   out_7351665863096111010[286] = 0;
   out_7351665863096111010[287] = 0;
   out_7351665863096111010[288] = 0;
   out_7351665863096111010[289] = 0;
   out_7351665863096111010[290] = 0;
   out_7351665863096111010[291] = 0;
   out_7351665863096111010[292] = 0;
   out_7351665863096111010[293] = 0;
   out_7351665863096111010[294] = 0;
   out_7351665863096111010[295] = 0;
   out_7351665863096111010[296] = 0;
   out_7351665863096111010[297] = 0;
   out_7351665863096111010[298] = 0;
   out_7351665863096111010[299] = 0;
   out_7351665863096111010[300] = 0;
   out_7351665863096111010[301] = 0;
   out_7351665863096111010[302] = 0;
   out_7351665863096111010[303] = 0;
   out_7351665863096111010[304] = 1;
   out_7351665863096111010[305] = 0;
   out_7351665863096111010[306] = 0;
   out_7351665863096111010[307] = 0;
   out_7351665863096111010[308] = 0;
   out_7351665863096111010[309] = 0;
   out_7351665863096111010[310] = 0;
   out_7351665863096111010[311] = 0;
   out_7351665863096111010[312] = 0;
   out_7351665863096111010[313] = 0;
   out_7351665863096111010[314] = 0;
   out_7351665863096111010[315] = 0;
   out_7351665863096111010[316] = 0;
   out_7351665863096111010[317] = 0;
   out_7351665863096111010[318] = 0;
   out_7351665863096111010[319] = 0;
   out_7351665863096111010[320] = 0;
   out_7351665863096111010[321] = 0;
   out_7351665863096111010[322] = 0;
   out_7351665863096111010[323] = 1;
}
void h_4(double *state, double *unused, double *out_3301009148839050449) {
   out_3301009148839050449[0] = state[6] + state[9];
   out_3301009148839050449[1] = state[7] + state[10];
   out_3301009148839050449[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_6872692752088360425) {
   out_6872692752088360425[0] = 0;
   out_6872692752088360425[1] = 0;
   out_6872692752088360425[2] = 0;
   out_6872692752088360425[3] = 0;
   out_6872692752088360425[4] = 0;
   out_6872692752088360425[5] = 0;
   out_6872692752088360425[6] = 1;
   out_6872692752088360425[7] = 0;
   out_6872692752088360425[8] = 0;
   out_6872692752088360425[9] = 1;
   out_6872692752088360425[10] = 0;
   out_6872692752088360425[11] = 0;
   out_6872692752088360425[12] = 0;
   out_6872692752088360425[13] = 0;
   out_6872692752088360425[14] = 0;
   out_6872692752088360425[15] = 0;
   out_6872692752088360425[16] = 0;
   out_6872692752088360425[17] = 0;
   out_6872692752088360425[18] = 0;
   out_6872692752088360425[19] = 0;
   out_6872692752088360425[20] = 0;
   out_6872692752088360425[21] = 0;
   out_6872692752088360425[22] = 0;
   out_6872692752088360425[23] = 0;
   out_6872692752088360425[24] = 0;
   out_6872692752088360425[25] = 1;
   out_6872692752088360425[26] = 0;
   out_6872692752088360425[27] = 0;
   out_6872692752088360425[28] = 1;
   out_6872692752088360425[29] = 0;
   out_6872692752088360425[30] = 0;
   out_6872692752088360425[31] = 0;
   out_6872692752088360425[32] = 0;
   out_6872692752088360425[33] = 0;
   out_6872692752088360425[34] = 0;
   out_6872692752088360425[35] = 0;
   out_6872692752088360425[36] = 0;
   out_6872692752088360425[37] = 0;
   out_6872692752088360425[38] = 0;
   out_6872692752088360425[39] = 0;
   out_6872692752088360425[40] = 0;
   out_6872692752088360425[41] = 0;
   out_6872692752088360425[42] = 0;
   out_6872692752088360425[43] = 0;
   out_6872692752088360425[44] = 1;
   out_6872692752088360425[45] = 0;
   out_6872692752088360425[46] = 0;
   out_6872692752088360425[47] = 1;
   out_6872692752088360425[48] = 0;
   out_6872692752088360425[49] = 0;
   out_6872692752088360425[50] = 0;
   out_6872692752088360425[51] = 0;
   out_6872692752088360425[52] = 0;
   out_6872692752088360425[53] = 0;
}
void h_10(double *state, double *unused, double *out_977489843187526232) {
   out_977489843187526232[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_977489843187526232[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_977489843187526232[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_6075288928525757853) {
   out_6075288928525757853[0] = 0;
   out_6075288928525757853[1] = 9.8100000000000005*cos(state[1]);
   out_6075288928525757853[2] = 0;
   out_6075288928525757853[3] = 0;
   out_6075288928525757853[4] = -state[8];
   out_6075288928525757853[5] = state[7];
   out_6075288928525757853[6] = 0;
   out_6075288928525757853[7] = state[5];
   out_6075288928525757853[8] = -state[4];
   out_6075288928525757853[9] = 0;
   out_6075288928525757853[10] = 0;
   out_6075288928525757853[11] = 0;
   out_6075288928525757853[12] = 1;
   out_6075288928525757853[13] = 0;
   out_6075288928525757853[14] = 0;
   out_6075288928525757853[15] = 1;
   out_6075288928525757853[16] = 0;
   out_6075288928525757853[17] = 0;
   out_6075288928525757853[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_6075288928525757853[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_6075288928525757853[20] = 0;
   out_6075288928525757853[21] = state[8];
   out_6075288928525757853[22] = 0;
   out_6075288928525757853[23] = -state[6];
   out_6075288928525757853[24] = -state[5];
   out_6075288928525757853[25] = 0;
   out_6075288928525757853[26] = state[3];
   out_6075288928525757853[27] = 0;
   out_6075288928525757853[28] = 0;
   out_6075288928525757853[29] = 0;
   out_6075288928525757853[30] = 0;
   out_6075288928525757853[31] = 1;
   out_6075288928525757853[32] = 0;
   out_6075288928525757853[33] = 0;
   out_6075288928525757853[34] = 1;
   out_6075288928525757853[35] = 0;
   out_6075288928525757853[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_6075288928525757853[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_6075288928525757853[38] = 0;
   out_6075288928525757853[39] = -state[7];
   out_6075288928525757853[40] = state[6];
   out_6075288928525757853[41] = 0;
   out_6075288928525757853[42] = state[4];
   out_6075288928525757853[43] = -state[3];
   out_6075288928525757853[44] = 0;
   out_6075288928525757853[45] = 0;
   out_6075288928525757853[46] = 0;
   out_6075288928525757853[47] = 0;
   out_6075288928525757853[48] = 0;
   out_6075288928525757853[49] = 0;
   out_6075288928525757853[50] = 1;
   out_6075288928525757853[51] = 0;
   out_6075288928525757853[52] = 0;
   out_6075288928525757853[53] = 1;
}
void h_13(double *state, double *unused, double *out_8626245788850028663) {
   out_8626245788850028663[0] = state[3];
   out_8626245788850028663[1] = state[4];
   out_8626245788850028663[2] = state[5];
}
void H_13(double *state, double *unused, double *out_3038937288785836401) {
   out_3038937288785836401[0] = 0;
   out_3038937288785836401[1] = 0;
   out_3038937288785836401[2] = 0;
   out_3038937288785836401[3] = 1;
   out_3038937288785836401[4] = 0;
   out_3038937288785836401[5] = 0;
   out_3038937288785836401[6] = 0;
   out_3038937288785836401[7] = 0;
   out_3038937288785836401[8] = 0;
   out_3038937288785836401[9] = 0;
   out_3038937288785836401[10] = 0;
   out_3038937288785836401[11] = 0;
   out_3038937288785836401[12] = 0;
   out_3038937288785836401[13] = 0;
   out_3038937288785836401[14] = 0;
   out_3038937288785836401[15] = 0;
   out_3038937288785836401[16] = 0;
   out_3038937288785836401[17] = 0;
   out_3038937288785836401[18] = 0;
   out_3038937288785836401[19] = 0;
   out_3038937288785836401[20] = 0;
   out_3038937288785836401[21] = 0;
   out_3038937288785836401[22] = 1;
   out_3038937288785836401[23] = 0;
   out_3038937288785836401[24] = 0;
   out_3038937288785836401[25] = 0;
   out_3038937288785836401[26] = 0;
   out_3038937288785836401[27] = 0;
   out_3038937288785836401[28] = 0;
   out_3038937288785836401[29] = 0;
   out_3038937288785836401[30] = 0;
   out_3038937288785836401[31] = 0;
   out_3038937288785836401[32] = 0;
   out_3038937288785836401[33] = 0;
   out_3038937288785836401[34] = 0;
   out_3038937288785836401[35] = 0;
   out_3038937288785836401[36] = 0;
   out_3038937288785836401[37] = 0;
   out_3038937288785836401[38] = 0;
   out_3038937288785836401[39] = 0;
   out_3038937288785836401[40] = 0;
   out_3038937288785836401[41] = 1;
   out_3038937288785836401[42] = 0;
   out_3038937288785836401[43] = 0;
   out_3038937288785836401[44] = 0;
   out_3038937288785836401[45] = 0;
   out_3038937288785836401[46] = 0;
   out_3038937288785836401[47] = 0;
   out_3038937288785836401[48] = 0;
   out_3038937288785836401[49] = 0;
   out_3038937288785836401[50] = 0;
   out_3038937288785836401[51] = 0;
   out_3038937288785836401[52] = 0;
   out_3038937288785836401[53] = 0;
}
void h_14(double *state, double *unused, double *out_4634144118535572398) {
   out_4634144118535572398[0] = state[6];
   out_4634144118535572398[1] = state[7];
   out_4634144118535572398[2] = state[8];
}
void H_14(double *state, double *unused, double *out_3789904319792988129) {
   out_3789904319792988129[0] = 0;
   out_3789904319792988129[1] = 0;
   out_3789904319792988129[2] = 0;
   out_3789904319792988129[3] = 0;
   out_3789904319792988129[4] = 0;
   out_3789904319792988129[5] = 0;
   out_3789904319792988129[6] = 1;
   out_3789904319792988129[7] = 0;
   out_3789904319792988129[8] = 0;
   out_3789904319792988129[9] = 0;
   out_3789904319792988129[10] = 0;
   out_3789904319792988129[11] = 0;
   out_3789904319792988129[12] = 0;
   out_3789904319792988129[13] = 0;
   out_3789904319792988129[14] = 0;
   out_3789904319792988129[15] = 0;
   out_3789904319792988129[16] = 0;
   out_3789904319792988129[17] = 0;
   out_3789904319792988129[18] = 0;
   out_3789904319792988129[19] = 0;
   out_3789904319792988129[20] = 0;
   out_3789904319792988129[21] = 0;
   out_3789904319792988129[22] = 0;
   out_3789904319792988129[23] = 0;
   out_3789904319792988129[24] = 0;
   out_3789904319792988129[25] = 1;
   out_3789904319792988129[26] = 0;
   out_3789904319792988129[27] = 0;
   out_3789904319792988129[28] = 0;
   out_3789904319792988129[29] = 0;
   out_3789904319792988129[30] = 0;
   out_3789904319792988129[31] = 0;
   out_3789904319792988129[32] = 0;
   out_3789904319792988129[33] = 0;
   out_3789904319792988129[34] = 0;
   out_3789904319792988129[35] = 0;
   out_3789904319792988129[36] = 0;
   out_3789904319792988129[37] = 0;
   out_3789904319792988129[38] = 0;
   out_3789904319792988129[39] = 0;
   out_3789904319792988129[40] = 0;
   out_3789904319792988129[41] = 0;
   out_3789904319792988129[42] = 0;
   out_3789904319792988129[43] = 0;
   out_3789904319792988129[44] = 1;
   out_3789904319792988129[45] = 0;
   out_3789904319792988129[46] = 0;
   out_3789904319792988129[47] = 0;
   out_3789904319792988129[48] = 0;
   out_3789904319792988129[49] = 0;
   out_3789904319792988129[50] = 0;
   out_3789904319792988129[51] = 0;
   out_3789904319792988129[52] = 0;
   out_3789904319792988129[53] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_4220870942759577714) {
  err_fun(nom_x, delta_x, out_4220870942759577714);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6691487759945560833) {
  inv_err_fun(nom_x, true_x, out_6691487759945560833);
}
void pose_H_mod_fun(double *state, double *out_5674745270205462321) {
  H_mod_fun(state, out_5674745270205462321);
}
void pose_f_fun(double *state, double dt, double *out_8950409172442745226) {
  f_fun(state,  dt, out_8950409172442745226);
}
void pose_F_fun(double *state, double dt, double *out_7351665863096111010) {
  F_fun(state,  dt, out_7351665863096111010);
}
void pose_h_4(double *state, double *unused, double *out_3301009148839050449) {
  h_4(state, unused, out_3301009148839050449);
}
void pose_H_4(double *state, double *unused, double *out_6872692752088360425) {
  H_4(state, unused, out_6872692752088360425);
}
void pose_h_10(double *state, double *unused, double *out_977489843187526232) {
  h_10(state, unused, out_977489843187526232);
}
void pose_H_10(double *state, double *unused, double *out_6075288928525757853) {
  H_10(state, unused, out_6075288928525757853);
}
void pose_h_13(double *state, double *unused, double *out_8626245788850028663) {
  h_13(state, unused, out_8626245788850028663);
}
void pose_H_13(double *state, double *unused, double *out_3038937288785836401) {
  H_13(state, unused, out_3038937288785836401);
}
void pose_h_14(double *state, double *unused, double *out_4634144118535572398) {
  h_14(state, unused, out_4634144118535572398);
}
void pose_H_14(double *state, double *unused, double *out_3789904319792988129) {
  H_14(state, unused, out_3789904319792988129);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
