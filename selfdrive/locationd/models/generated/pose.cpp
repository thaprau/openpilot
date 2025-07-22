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
void err_fun(double *nom_x, double *delta_x, double *out_4190350983600540570) {
   out_4190350983600540570[0] = delta_x[0] + nom_x[0];
   out_4190350983600540570[1] = delta_x[1] + nom_x[1];
   out_4190350983600540570[2] = delta_x[2] + nom_x[2];
   out_4190350983600540570[3] = delta_x[3] + nom_x[3];
   out_4190350983600540570[4] = delta_x[4] + nom_x[4];
   out_4190350983600540570[5] = delta_x[5] + nom_x[5];
   out_4190350983600540570[6] = delta_x[6] + nom_x[6];
   out_4190350983600540570[7] = delta_x[7] + nom_x[7];
   out_4190350983600540570[8] = delta_x[8] + nom_x[8];
   out_4190350983600540570[9] = delta_x[9] + nom_x[9];
   out_4190350983600540570[10] = delta_x[10] + nom_x[10];
   out_4190350983600540570[11] = delta_x[11] + nom_x[11];
   out_4190350983600540570[12] = delta_x[12] + nom_x[12];
   out_4190350983600540570[13] = delta_x[13] + nom_x[13];
   out_4190350983600540570[14] = delta_x[14] + nom_x[14];
   out_4190350983600540570[15] = delta_x[15] + nom_x[15];
   out_4190350983600540570[16] = delta_x[16] + nom_x[16];
   out_4190350983600540570[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6497364289119633418) {
   out_6497364289119633418[0] = -nom_x[0] + true_x[0];
   out_6497364289119633418[1] = -nom_x[1] + true_x[1];
   out_6497364289119633418[2] = -nom_x[2] + true_x[2];
   out_6497364289119633418[3] = -nom_x[3] + true_x[3];
   out_6497364289119633418[4] = -nom_x[4] + true_x[4];
   out_6497364289119633418[5] = -nom_x[5] + true_x[5];
   out_6497364289119633418[6] = -nom_x[6] + true_x[6];
   out_6497364289119633418[7] = -nom_x[7] + true_x[7];
   out_6497364289119633418[8] = -nom_x[8] + true_x[8];
   out_6497364289119633418[9] = -nom_x[9] + true_x[9];
   out_6497364289119633418[10] = -nom_x[10] + true_x[10];
   out_6497364289119633418[11] = -nom_x[11] + true_x[11];
   out_6497364289119633418[12] = -nom_x[12] + true_x[12];
   out_6497364289119633418[13] = -nom_x[13] + true_x[13];
   out_6497364289119633418[14] = -nom_x[14] + true_x[14];
   out_6497364289119633418[15] = -nom_x[15] + true_x[15];
   out_6497364289119633418[16] = -nom_x[16] + true_x[16];
   out_6497364289119633418[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4927489086149144796) {
   out_4927489086149144796[0] = 1.0;
   out_4927489086149144796[1] = 0.0;
   out_4927489086149144796[2] = 0.0;
   out_4927489086149144796[3] = 0.0;
   out_4927489086149144796[4] = 0.0;
   out_4927489086149144796[5] = 0.0;
   out_4927489086149144796[6] = 0.0;
   out_4927489086149144796[7] = 0.0;
   out_4927489086149144796[8] = 0.0;
   out_4927489086149144796[9] = 0.0;
   out_4927489086149144796[10] = 0.0;
   out_4927489086149144796[11] = 0.0;
   out_4927489086149144796[12] = 0.0;
   out_4927489086149144796[13] = 0.0;
   out_4927489086149144796[14] = 0.0;
   out_4927489086149144796[15] = 0.0;
   out_4927489086149144796[16] = 0.0;
   out_4927489086149144796[17] = 0.0;
   out_4927489086149144796[18] = 0.0;
   out_4927489086149144796[19] = 1.0;
   out_4927489086149144796[20] = 0.0;
   out_4927489086149144796[21] = 0.0;
   out_4927489086149144796[22] = 0.0;
   out_4927489086149144796[23] = 0.0;
   out_4927489086149144796[24] = 0.0;
   out_4927489086149144796[25] = 0.0;
   out_4927489086149144796[26] = 0.0;
   out_4927489086149144796[27] = 0.0;
   out_4927489086149144796[28] = 0.0;
   out_4927489086149144796[29] = 0.0;
   out_4927489086149144796[30] = 0.0;
   out_4927489086149144796[31] = 0.0;
   out_4927489086149144796[32] = 0.0;
   out_4927489086149144796[33] = 0.0;
   out_4927489086149144796[34] = 0.0;
   out_4927489086149144796[35] = 0.0;
   out_4927489086149144796[36] = 0.0;
   out_4927489086149144796[37] = 0.0;
   out_4927489086149144796[38] = 1.0;
   out_4927489086149144796[39] = 0.0;
   out_4927489086149144796[40] = 0.0;
   out_4927489086149144796[41] = 0.0;
   out_4927489086149144796[42] = 0.0;
   out_4927489086149144796[43] = 0.0;
   out_4927489086149144796[44] = 0.0;
   out_4927489086149144796[45] = 0.0;
   out_4927489086149144796[46] = 0.0;
   out_4927489086149144796[47] = 0.0;
   out_4927489086149144796[48] = 0.0;
   out_4927489086149144796[49] = 0.0;
   out_4927489086149144796[50] = 0.0;
   out_4927489086149144796[51] = 0.0;
   out_4927489086149144796[52] = 0.0;
   out_4927489086149144796[53] = 0.0;
   out_4927489086149144796[54] = 0.0;
   out_4927489086149144796[55] = 0.0;
   out_4927489086149144796[56] = 0.0;
   out_4927489086149144796[57] = 1.0;
   out_4927489086149144796[58] = 0.0;
   out_4927489086149144796[59] = 0.0;
   out_4927489086149144796[60] = 0.0;
   out_4927489086149144796[61] = 0.0;
   out_4927489086149144796[62] = 0.0;
   out_4927489086149144796[63] = 0.0;
   out_4927489086149144796[64] = 0.0;
   out_4927489086149144796[65] = 0.0;
   out_4927489086149144796[66] = 0.0;
   out_4927489086149144796[67] = 0.0;
   out_4927489086149144796[68] = 0.0;
   out_4927489086149144796[69] = 0.0;
   out_4927489086149144796[70] = 0.0;
   out_4927489086149144796[71] = 0.0;
   out_4927489086149144796[72] = 0.0;
   out_4927489086149144796[73] = 0.0;
   out_4927489086149144796[74] = 0.0;
   out_4927489086149144796[75] = 0.0;
   out_4927489086149144796[76] = 1.0;
   out_4927489086149144796[77] = 0.0;
   out_4927489086149144796[78] = 0.0;
   out_4927489086149144796[79] = 0.0;
   out_4927489086149144796[80] = 0.0;
   out_4927489086149144796[81] = 0.0;
   out_4927489086149144796[82] = 0.0;
   out_4927489086149144796[83] = 0.0;
   out_4927489086149144796[84] = 0.0;
   out_4927489086149144796[85] = 0.0;
   out_4927489086149144796[86] = 0.0;
   out_4927489086149144796[87] = 0.0;
   out_4927489086149144796[88] = 0.0;
   out_4927489086149144796[89] = 0.0;
   out_4927489086149144796[90] = 0.0;
   out_4927489086149144796[91] = 0.0;
   out_4927489086149144796[92] = 0.0;
   out_4927489086149144796[93] = 0.0;
   out_4927489086149144796[94] = 0.0;
   out_4927489086149144796[95] = 1.0;
   out_4927489086149144796[96] = 0.0;
   out_4927489086149144796[97] = 0.0;
   out_4927489086149144796[98] = 0.0;
   out_4927489086149144796[99] = 0.0;
   out_4927489086149144796[100] = 0.0;
   out_4927489086149144796[101] = 0.0;
   out_4927489086149144796[102] = 0.0;
   out_4927489086149144796[103] = 0.0;
   out_4927489086149144796[104] = 0.0;
   out_4927489086149144796[105] = 0.0;
   out_4927489086149144796[106] = 0.0;
   out_4927489086149144796[107] = 0.0;
   out_4927489086149144796[108] = 0.0;
   out_4927489086149144796[109] = 0.0;
   out_4927489086149144796[110] = 0.0;
   out_4927489086149144796[111] = 0.0;
   out_4927489086149144796[112] = 0.0;
   out_4927489086149144796[113] = 0.0;
   out_4927489086149144796[114] = 1.0;
   out_4927489086149144796[115] = 0.0;
   out_4927489086149144796[116] = 0.0;
   out_4927489086149144796[117] = 0.0;
   out_4927489086149144796[118] = 0.0;
   out_4927489086149144796[119] = 0.0;
   out_4927489086149144796[120] = 0.0;
   out_4927489086149144796[121] = 0.0;
   out_4927489086149144796[122] = 0.0;
   out_4927489086149144796[123] = 0.0;
   out_4927489086149144796[124] = 0.0;
   out_4927489086149144796[125] = 0.0;
   out_4927489086149144796[126] = 0.0;
   out_4927489086149144796[127] = 0.0;
   out_4927489086149144796[128] = 0.0;
   out_4927489086149144796[129] = 0.0;
   out_4927489086149144796[130] = 0.0;
   out_4927489086149144796[131] = 0.0;
   out_4927489086149144796[132] = 0.0;
   out_4927489086149144796[133] = 1.0;
   out_4927489086149144796[134] = 0.0;
   out_4927489086149144796[135] = 0.0;
   out_4927489086149144796[136] = 0.0;
   out_4927489086149144796[137] = 0.0;
   out_4927489086149144796[138] = 0.0;
   out_4927489086149144796[139] = 0.0;
   out_4927489086149144796[140] = 0.0;
   out_4927489086149144796[141] = 0.0;
   out_4927489086149144796[142] = 0.0;
   out_4927489086149144796[143] = 0.0;
   out_4927489086149144796[144] = 0.0;
   out_4927489086149144796[145] = 0.0;
   out_4927489086149144796[146] = 0.0;
   out_4927489086149144796[147] = 0.0;
   out_4927489086149144796[148] = 0.0;
   out_4927489086149144796[149] = 0.0;
   out_4927489086149144796[150] = 0.0;
   out_4927489086149144796[151] = 0.0;
   out_4927489086149144796[152] = 1.0;
   out_4927489086149144796[153] = 0.0;
   out_4927489086149144796[154] = 0.0;
   out_4927489086149144796[155] = 0.0;
   out_4927489086149144796[156] = 0.0;
   out_4927489086149144796[157] = 0.0;
   out_4927489086149144796[158] = 0.0;
   out_4927489086149144796[159] = 0.0;
   out_4927489086149144796[160] = 0.0;
   out_4927489086149144796[161] = 0.0;
   out_4927489086149144796[162] = 0.0;
   out_4927489086149144796[163] = 0.0;
   out_4927489086149144796[164] = 0.0;
   out_4927489086149144796[165] = 0.0;
   out_4927489086149144796[166] = 0.0;
   out_4927489086149144796[167] = 0.0;
   out_4927489086149144796[168] = 0.0;
   out_4927489086149144796[169] = 0.0;
   out_4927489086149144796[170] = 0.0;
   out_4927489086149144796[171] = 1.0;
   out_4927489086149144796[172] = 0.0;
   out_4927489086149144796[173] = 0.0;
   out_4927489086149144796[174] = 0.0;
   out_4927489086149144796[175] = 0.0;
   out_4927489086149144796[176] = 0.0;
   out_4927489086149144796[177] = 0.0;
   out_4927489086149144796[178] = 0.0;
   out_4927489086149144796[179] = 0.0;
   out_4927489086149144796[180] = 0.0;
   out_4927489086149144796[181] = 0.0;
   out_4927489086149144796[182] = 0.0;
   out_4927489086149144796[183] = 0.0;
   out_4927489086149144796[184] = 0.0;
   out_4927489086149144796[185] = 0.0;
   out_4927489086149144796[186] = 0.0;
   out_4927489086149144796[187] = 0.0;
   out_4927489086149144796[188] = 0.0;
   out_4927489086149144796[189] = 0.0;
   out_4927489086149144796[190] = 1.0;
   out_4927489086149144796[191] = 0.0;
   out_4927489086149144796[192] = 0.0;
   out_4927489086149144796[193] = 0.0;
   out_4927489086149144796[194] = 0.0;
   out_4927489086149144796[195] = 0.0;
   out_4927489086149144796[196] = 0.0;
   out_4927489086149144796[197] = 0.0;
   out_4927489086149144796[198] = 0.0;
   out_4927489086149144796[199] = 0.0;
   out_4927489086149144796[200] = 0.0;
   out_4927489086149144796[201] = 0.0;
   out_4927489086149144796[202] = 0.0;
   out_4927489086149144796[203] = 0.0;
   out_4927489086149144796[204] = 0.0;
   out_4927489086149144796[205] = 0.0;
   out_4927489086149144796[206] = 0.0;
   out_4927489086149144796[207] = 0.0;
   out_4927489086149144796[208] = 0.0;
   out_4927489086149144796[209] = 1.0;
   out_4927489086149144796[210] = 0.0;
   out_4927489086149144796[211] = 0.0;
   out_4927489086149144796[212] = 0.0;
   out_4927489086149144796[213] = 0.0;
   out_4927489086149144796[214] = 0.0;
   out_4927489086149144796[215] = 0.0;
   out_4927489086149144796[216] = 0.0;
   out_4927489086149144796[217] = 0.0;
   out_4927489086149144796[218] = 0.0;
   out_4927489086149144796[219] = 0.0;
   out_4927489086149144796[220] = 0.0;
   out_4927489086149144796[221] = 0.0;
   out_4927489086149144796[222] = 0.0;
   out_4927489086149144796[223] = 0.0;
   out_4927489086149144796[224] = 0.0;
   out_4927489086149144796[225] = 0.0;
   out_4927489086149144796[226] = 0.0;
   out_4927489086149144796[227] = 0.0;
   out_4927489086149144796[228] = 1.0;
   out_4927489086149144796[229] = 0.0;
   out_4927489086149144796[230] = 0.0;
   out_4927489086149144796[231] = 0.0;
   out_4927489086149144796[232] = 0.0;
   out_4927489086149144796[233] = 0.0;
   out_4927489086149144796[234] = 0.0;
   out_4927489086149144796[235] = 0.0;
   out_4927489086149144796[236] = 0.0;
   out_4927489086149144796[237] = 0.0;
   out_4927489086149144796[238] = 0.0;
   out_4927489086149144796[239] = 0.0;
   out_4927489086149144796[240] = 0.0;
   out_4927489086149144796[241] = 0.0;
   out_4927489086149144796[242] = 0.0;
   out_4927489086149144796[243] = 0.0;
   out_4927489086149144796[244] = 0.0;
   out_4927489086149144796[245] = 0.0;
   out_4927489086149144796[246] = 0.0;
   out_4927489086149144796[247] = 1.0;
   out_4927489086149144796[248] = 0.0;
   out_4927489086149144796[249] = 0.0;
   out_4927489086149144796[250] = 0.0;
   out_4927489086149144796[251] = 0.0;
   out_4927489086149144796[252] = 0.0;
   out_4927489086149144796[253] = 0.0;
   out_4927489086149144796[254] = 0.0;
   out_4927489086149144796[255] = 0.0;
   out_4927489086149144796[256] = 0.0;
   out_4927489086149144796[257] = 0.0;
   out_4927489086149144796[258] = 0.0;
   out_4927489086149144796[259] = 0.0;
   out_4927489086149144796[260] = 0.0;
   out_4927489086149144796[261] = 0.0;
   out_4927489086149144796[262] = 0.0;
   out_4927489086149144796[263] = 0.0;
   out_4927489086149144796[264] = 0.0;
   out_4927489086149144796[265] = 0.0;
   out_4927489086149144796[266] = 1.0;
   out_4927489086149144796[267] = 0.0;
   out_4927489086149144796[268] = 0.0;
   out_4927489086149144796[269] = 0.0;
   out_4927489086149144796[270] = 0.0;
   out_4927489086149144796[271] = 0.0;
   out_4927489086149144796[272] = 0.0;
   out_4927489086149144796[273] = 0.0;
   out_4927489086149144796[274] = 0.0;
   out_4927489086149144796[275] = 0.0;
   out_4927489086149144796[276] = 0.0;
   out_4927489086149144796[277] = 0.0;
   out_4927489086149144796[278] = 0.0;
   out_4927489086149144796[279] = 0.0;
   out_4927489086149144796[280] = 0.0;
   out_4927489086149144796[281] = 0.0;
   out_4927489086149144796[282] = 0.0;
   out_4927489086149144796[283] = 0.0;
   out_4927489086149144796[284] = 0.0;
   out_4927489086149144796[285] = 1.0;
   out_4927489086149144796[286] = 0.0;
   out_4927489086149144796[287] = 0.0;
   out_4927489086149144796[288] = 0.0;
   out_4927489086149144796[289] = 0.0;
   out_4927489086149144796[290] = 0.0;
   out_4927489086149144796[291] = 0.0;
   out_4927489086149144796[292] = 0.0;
   out_4927489086149144796[293] = 0.0;
   out_4927489086149144796[294] = 0.0;
   out_4927489086149144796[295] = 0.0;
   out_4927489086149144796[296] = 0.0;
   out_4927489086149144796[297] = 0.0;
   out_4927489086149144796[298] = 0.0;
   out_4927489086149144796[299] = 0.0;
   out_4927489086149144796[300] = 0.0;
   out_4927489086149144796[301] = 0.0;
   out_4927489086149144796[302] = 0.0;
   out_4927489086149144796[303] = 0.0;
   out_4927489086149144796[304] = 1.0;
   out_4927489086149144796[305] = 0.0;
   out_4927489086149144796[306] = 0.0;
   out_4927489086149144796[307] = 0.0;
   out_4927489086149144796[308] = 0.0;
   out_4927489086149144796[309] = 0.0;
   out_4927489086149144796[310] = 0.0;
   out_4927489086149144796[311] = 0.0;
   out_4927489086149144796[312] = 0.0;
   out_4927489086149144796[313] = 0.0;
   out_4927489086149144796[314] = 0.0;
   out_4927489086149144796[315] = 0.0;
   out_4927489086149144796[316] = 0.0;
   out_4927489086149144796[317] = 0.0;
   out_4927489086149144796[318] = 0.0;
   out_4927489086149144796[319] = 0.0;
   out_4927489086149144796[320] = 0.0;
   out_4927489086149144796[321] = 0.0;
   out_4927489086149144796[322] = 0.0;
   out_4927489086149144796[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5720185918353944946) {
   out_5720185918353944946[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5720185918353944946[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5720185918353944946[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5720185918353944946[3] = dt*state[12] + state[3];
   out_5720185918353944946[4] = dt*state[13] + state[4];
   out_5720185918353944946[5] = dt*state[14] + state[5];
   out_5720185918353944946[6] = state[6];
   out_5720185918353944946[7] = state[7];
   out_5720185918353944946[8] = state[8];
   out_5720185918353944946[9] = state[9];
   out_5720185918353944946[10] = state[10];
   out_5720185918353944946[11] = state[11];
   out_5720185918353944946[12] = state[12];
   out_5720185918353944946[13] = state[13];
   out_5720185918353944946[14] = state[14];
   out_5720185918353944946[15] = state[15];
   out_5720185918353944946[16] = state[16];
   out_5720185918353944946[17] = state[17];
}
void F_fun(double *state, double dt, double *out_9170706574201723667) {
   out_9170706574201723667[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_9170706574201723667[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_9170706574201723667[2] = 0;
   out_9170706574201723667[3] = 0;
   out_9170706574201723667[4] = 0;
   out_9170706574201723667[5] = 0;
   out_9170706574201723667[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_9170706574201723667[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_9170706574201723667[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_9170706574201723667[9] = 0;
   out_9170706574201723667[10] = 0;
   out_9170706574201723667[11] = 0;
   out_9170706574201723667[12] = 0;
   out_9170706574201723667[13] = 0;
   out_9170706574201723667[14] = 0;
   out_9170706574201723667[15] = 0;
   out_9170706574201723667[16] = 0;
   out_9170706574201723667[17] = 0;
   out_9170706574201723667[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_9170706574201723667[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_9170706574201723667[20] = 0;
   out_9170706574201723667[21] = 0;
   out_9170706574201723667[22] = 0;
   out_9170706574201723667[23] = 0;
   out_9170706574201723667[24] = 0;
   out_9170706574201723667[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_9170706574201723667[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_9170706574201723667[27] = 0;
   out_9170706574201723667[28] = 0;
   out_9170706574201723667[29] = 0;
   out_9170706574201723667[30] = 0;
   out_9170706574201723667[31] = 0;
   out_9170706574201723667[32] = 0;
   out_9170706574201723667[33] = 0;
   out_9170706574201723667[34] = 0;
   out_9170706574201723667[35] = 0;
   out_9170706574201723667[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_9170706574201723667[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_9170706574201723667[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_9170706574201723667[39] = 0;
   out_9170706574201723667[40] = 0;
   out_9170706574201723667[41] = 0;
   out_9170706574201723667[42] = 0;
   out_9170706574201723667[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_9170706574201723667[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_9170706574201723667[45] = 0;
   out_9170706574201723667[46] = 0;
   out_9170706574201723667[47] = 0;
   out_9170706574201723667[48] = 0;
   out_9170706574201723667[49] = 0;
   out_9170706574201723667[50] = 0;
   out_9170706574201723667[51] = 0;
   out_9170706574201723667[52] = 0;
   out_9170706574201723667[53] = 0;
   out_9170706574201723667[54] = 0;
   out_9170706574201723667[55] = 0;
   out_9170706574201723667[56] = 0;
   out_9170706574201723667[57] = 1;
   out_9170706574201723667[58] = 0;
   out_9170706574201723667[59] = 0;
   out_9170706574201723667[60] = 0;
   out_9170706574201723667[61] = 0;
   out_9170706574201723667[62] = 0;
   out_9170706574201723667[63] = 0;
   out_9170706574201723667[64] = 0;
   out_9170706574201723667[65] = 0;
   out_9170706574201723667[66] = dt;
   out_9170706574201723667[67] = 0;
   out_9170706574201723667[68] = 0;
   out_9170706574201723667[69] = 0;
   out_9170706574201723667[70] = 0;
   out_9170706574201723667[71] = 0;
   out_9170706574201723667[72] = 0;
   out_9170706574201723667[73] = 0;
   out_9170706574201723667[74] = 0;
   out_9170706574201723667[75] = 0;
   out_9170706574201723667[76] = 1;
   out_9170706574201723667[77] = 0;
   out_9170706574201723667[78] = 0;
   out_9170706574201723667[79] = 0;
   out_9170706574201723667[80] = 0;
   out_9170706574201723667[81] = 0;
   out_9170706574201723667[82] = 0;
   out_9170706574201723667[83] = 0;
   out_9170706574201723667[84] = 0;
   out_9170706574201723667[85] = dt;
   out_9170706574201723667[86] = 0;
   out_9170706574201723667[87] = 0;
   out_9170706574201723667[88] = 0;
   out_9170706574201723667[89] = 0;
   out_9170706574201723667[90] = 0;
   out_9170706574201723667[91] = 0;
   out_9170706574201723667[92] = 0;
   out_9170706574201723667[93] = 0;
   out_9170706574201723667[94] = 0;
   out_9170706574201723667[95] = 1;
   out_9170706574201723667[96] = 0;
   out_9170706574201723667[97] = 0;
   out_9170706574201723667[98] = 0;
   out_9170706574201723667[99] = 0;
   out_9170706574201723667[100] = 0;
   out_9170706574201723667[101] = 0;
   out_9170706574201723667[102] = 0;
   out_9170706574201723667[103] = 0;
   out_9170706574201723667[104] = dt;
   out_9170706574201723667[105] = 0;
   out_9170706574201723667[106] = 0;
   out_9170706574201723667[107] = 0;
   out_9170706574201723667[108] = 0;
   out_9170706574201723667[109] = 0;
   out_9170706574201723667[110] = 0;
   out_9170706574201723667[111] = 0;
   out_9170706574201723667[112] = 0;
   out_9170706574201723667[113] = 0;
   out_9170706574201723667[114] = 1;
   out_9170706574201723667[115] = 0;
   out_9170706574201723667[116] = 0;
   out_9170706574201723667[117] = 0;
   out_9170706574201723667[118] = 0;
   out_9170706574201723667[119] = 0;
   out_9170706574201723667[120] = 0;
   out_9170706574201723667[121] = 0;
   out_9170706574201723667[122] = 0;
   out_9170706574201723667[123] = 0;
   out_9170706574201723667[124] = 0;
   out_9170706574201723667[125] = 0;
   out_9170706574201723667[126] = 0;
   out_9170706574201723667[127] = 0;
   out_9170706574201723667[128] = 0;
   out_9170706574201723667[129] = 0;
   out_9170706574201723667[130] = 0;
   out_9170706574201723667[131] = 0;
   out_9170706574201723667[132] = 0;
   out_9170706574201723667[133] = 1;
   out_9170706574201723667[134] = 0;
   out_9170706574201723667[135] = 0;
   out_9170706574201723667[136] = 0;
   out_9170706574201723667[137] = 0;
   out_9170706574201723667[138] = 0;
   out_9170706574201723667[139] = 0;
   out_9170706574201723667[140] = 0;
   out_9170706574201723667[141] = 0;
   out_9170706574201723667[142] = 0;
   out_9170706574201723667[143] = 0;
   out_9170706574201723667[144] = 0;
   out_9170706574201723667[145] = 0;
   out_9170706574201723667[146] = 0;
   out_9170706574201723667[147] = 0;
   out_9170706574201723667[148] = 0;
   out_9170706574201723667[149] = 0;
   out_9170706574201723667[150] = 0;
   out_9170706574201723667[151] = 0;
   out_9170706574201723667[152] = 1;
   out_9170706574201723667[153] = 0;
   out_9170706574201723667[154] = 0;
   out_9170706574201723667[155] = 0;
   out_9170706574201723667[156] = 0;
   out_9170706574201723667[157] = 0;
   out_9170706574201723667[158] = 0;
   out_9170706574201723667[159] = 0;
   out_9170706574201723667[160] = 0;
   out_9170706574201723667[161] = 0;
   out_9170706574201723667[162] = 0;
   out_9170706574201723667[163] = 0;
   out_9170706574201723667[164] = 0;
   out_9170706574201723667[165] = 0;
   out_9170706574201723667[166] = 0;
   out_9170706574201723667[167] = 0;
   out_9170706574201723667[168] = 0;
   out_9170706574201723667[169] = 0;
   out_9170706574201723667[170] = 0;
   out_9170706574201723667[171] = 1;
   out_9170706574201723667[172] = 0;
   out_9170706574201723667[173] = 0;
   out_9170706574201723667[174] = 0;
   out_9170706574201723667[175] = 0;
   out_9170706574201723667[176] = 0;
   out_9170706574201723667[177] = 0;
   out_9170706574201723667[178] = 0;
   out_9170706574201723667[179] = 0;
   out_9170706574201723667[180] = 0;
   out_9170706574201723667[181] = 0;
   out_9170706574201723667[182] = 0;
   out_9170706574201723667[183] = 0;
   out_9170706574201723667[184] = 0;
   out_9170706574201723667[185] = 0;
   out_9170706574201723667[186] = 0;
   out_9170706574201723667[187] = 0;
   out_9170706574201723667[188] = 0;
   out_9170706574201723667[189] = 0;
   out_9170706574201723667[190] = 1;
   out_9170706574201723667[191] = 0;
   out_9170706574201723667[192] = 0;
   out_9170706574201723667[193] = 0;
   out_9170706574201723667[194] = 0;
   out_9170706574201723667[195] = 0;
   out_9170706574201723667[196] = 0;
   out_9170706574201723667[197] = 0;
   out_9170706574201723667[198] = 0;
   out_9170706574201723667[199] = 0;
   out_9170706574201723667[200] = 0;
   out_9170706574201723667[201] = 0;
   out_9170706574201723667[202] = 0;
   out_9170706574201723667[203] = 0;
   out_9170706574201723667[204] = 0;
   out_9170706574201723667[205] = 0;
   out_9170706574201723667[206] = 0;
   out_9170706574201723667[207] = 0;
   out_9170706574201723667[208] = 0;
   out_9170706574201723667[209] = 1;
   out_9170706574201723667[210] = 0;
   out_9170706574201723667[211] = 0;
   out_9170706574201723667[212] = 0;
   out_9170706574201723667[213] = 0;
   out_9170706574201723667[214] = 0;
   out_9170706574201723667[215] = 0;
   out_9170706574201723667[216] = 0;
   out_9170706574201723667[217] = 0;
   out_9170706574201723667[218] = 0;
   out_9170706574201723667[219] = 0;
   out_9170706574201723667[220] = 0;
   out_9170706574201723667[221] = 0;
   out_9170706574201723667[222] = 0;
   out_9170706574201723667[223] = 0;
   out_9170706574201723667[224] = 0;
   out_9170706574201723667[225] = 0;
   out_9170706574201723667[226] = 0;
   out_9170706574201723667[227] = 0;
   out_9170706574201723667[228] = 1;
   out_9170706574201723667[229] = 0;
   out_9170706574201723667[230] = 0;
   out_9170706574201723667[231] = 0;
   out_9170706574201723667[232] = 0;
   out_9170706574201723667[233] = 0;
   out_9170706574201723667[234] = 0;
   out_9170706574201723667[235] = 0;
   out_9170706574201723667[236] = 0;
   out_9170706574201723667[237] = 0;
   out_9170706574201723667[238] = 0;
   out_9170706574201723667[239] = 0;
   out_9170706574201723667[240] = 0;
   out_9170706574201723667[241] = 0;
   out_9170706574201723667[242] = 0;
   out_9170706574201723667[243] = 0;
   out_9170706574201723667[244] = 0;
   out_9170706574201723667[245] = 0;
   out_9170706574201723667[246] = 0;
   out_9170706574201723667[247] = 1;
   out_9170706574201723667[248] = 0;
   out_9170706574201723667[249] = 0;
   out_9170706574201723667[250] = 0;
   out_9170706574201723667[251] = 0;
   out_9170706574201723667[252] = 0;
   out_9170706574201723667[253] = 0;
   out_9170706574201723667[254] = 0;
   out_9170706574201723667[255] = 0;
   out_9170706574201723667[256] = 0;
   out_9170706574201723667[257] = 0;
   out_9170706574201723667[258] = 0;
   out_9170706574201723667[259] = 0;
   out_9170706574201723667[260] = 0;
   out_9170706574201723667[261] = 0;
   out_9170706574201723667[262] = 0;
   out_9170706574201723667[263] = 0;
   out_9170706574201723667[264] = 0;
   out_9170706574201723667[265] = 0;
   out_9170706574201723667[266] = 1;
   out_9170706574201723667[267] = 0;
   out_9170706574201723667[268] = 0;
   out_9170706574201723667[269] = 0;
   out_9170706574201723667[270] = 0;
   out_9170706574201723667[271] = 0;
   out_9170706574201723667[272] = 0;
   out_9170706574201723667[273] = 0;
   out_9170706574201723667[274] = 0;
   out_9170706574201723667[275] = 0;
   out_9170706574201723667[276] = 0;
   out_9170706574201723667[277] = 0;
   out_9170706574201723667[278] = 0;
   out_9170706574201723667[279] = 0;
   out_9170706574201723667[280] = 0;
   out_9170706574201723667[281] = 0;
   out_9170706574201723667[282] = 0;
   out_9170706574201723667[283] = 0;
   out_9170706574201723667[284] = 0;
   out_9170706574201723667[285] = 1;
   out_9170706574201723667[286] = 0;
   out_9170706574201723667[287] = 0;
   out_9170706574201723667[288] = 0;
   out_9170706574201723667[289] = 0;
   out_9170706574201723667[290] = 0;
   out_9170706574201723667[291] = 0;
   out_9170706574201723667[292] = 0;
   out_9170706574201723667[293] = 0;
   out_9170706574201723667[294] = 0;
   out_9170706574201723667[295] = 0;
   out_9170706574201723667[296] = 0;
   out_9170706574201723667[297] = 0;
   out_9170706574201723667[298] = 0;
   out_9170706574201723667[299] = 0;
   out_9170706574201723667[300] = 0;
   out_9170706574201723667[301] = 0;
   out_9170706574201723667[302] = 0;
   out_9170706574201723667[303] = 0;
   out_9170706574201723667[304] = 1;
   out_9170706574201723667[305] = 0;
   out_9170706574201723667[306] = 0;
   out_9170706574201723667[307] = 0;
   out_9170706574201723667[308] = 0;
   out_9170706574201723667[309] = 0;
   out_9170706574201723667[310] = 0;
   out_9170706574201723667[311] = 0;
   out_9170706574201723667[312] = 0;
   out_9170706574201723667[313] = 0;
   out_9170706574201723667[314] = 0;
   out_9170706574201723667[315] = 0;
   out_9170706574201723667[316] = 0;
   out_9170706574201723667[317] = 0;
   out_9170706574201723667[318] = 0;
   out_9170706574201723667[319] = 0;
   out_9170706574201723667[320] = 0;
   out_9170706574201723667[321] = 0;
   out_9170706574201723667[322] = 0;
   out_9170706574201723667[323] = 1;
}
void h_4(double *state, double *unused, double *out_9123423313710806344) {
   out_9123423313710806344[0] = state[6] + state[9];
   out_9123423313710806344[1] = state[7] + state[10];
   out_9123423313710806344[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_920592720602813925) {
   out_920592720602813925[0] = 0;
   out_920592720602813925[1] = 0;
   out_920592720602813925[2] = 0;
   out_920592720602813925[3] = 0;
   out_920592720602813925[4] = 0;
   out_920592720602813925[5] = 0;
   out_920592720602813925[6] = 1;
   out_920592720602813925[7] = 0;
   out_920592720602813925[8] = 0;
   out_920592720602813925[9] = 1;
   out_920592720602813925[10] = 0;
   out_920592720602813925[11] = 0;
   out_920592720602813925[12] = 0;
   out_920592720602813925[13] = 0;
   out_920592720602813925[14] = 0;
   out_920592720602813925[15] = 0;
   out_920592720602813925[16] = 0;
   out_920592720602813925[17] = 0;
   out_920592720602813925[18] = 0;
   out_920592720602813925[19] = 0;
   out_920592720602813925[20] = 0;
   out_920592720602813925[21] = 0;
   out_920592720602813925[22] = 0;
   out_920592720602813925[23] = 0;
   out_920592720602813925[24] = 0;
   out_920592720602813925[25] = 1;
   out_920592720602813925[26] = 0;
   out_920592720602813925[27] = 0;
   out_920592720602813925[28] = 1;
   out_920592720602813925[29] = 0;
   out_920592720602813925[30] = 0;
   out_920592720602813925[31] = 0;
   out_920592720602813925[32] = 0;
   out_920592720602813925[33] = 0;
   out_920592720602813925[34] = 0;
   out_920592720602813925[35] = 0;
   out_920592720602813925[36] = 0;
   out_920592720602813925[37] = 0;
   out_920592720602813925[38] = 0;
   out_920592720602813925[39] = 0;
   out_920592720602813925[40] = 0;
   out_920592720602813925[41] = 0;
   out_920592720602813925[42] = 0;
   out_920592720602813925[43] = 0;
   out_920592720602813925[44] = 1;
   out_920592720602813925[45] = 0;
   out_920592720602813925[46] = 0;
   out_920592720602813925[47] = 1;
   out_920592720602813925[48] = 0;
   out_920592720602813925[49] = 0;
   out_920592720602813925[50] = 0;
   out_920592720602813925[51] = 0;
   out_920592720602813925[52] = 0;
   out_920592720602813925[53] = 0;
}
void h_10(double *state, double *unused, double *out_2741809755357044354) {
   out_2741809755357044354[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2741809755357044354[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2741809755357044354[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_2432780480652285733) {
   out_2432780480652285733[0] = 0;
   out_2432780480652285733[1] = 9.8100000000000005*cos(state[1]);
   out_2432780480652285733[2] = 0;
   out_2432780480652285733[3] = 0;
   out_2432780480652285733[4] = -state[8];
   out_2432780480652285733[5] = state[7];
   out_2432780480652285733[6] = 0;
   out_2432780480652285733[7] = state[5];
   out_2432780480652285733[8] = -state[4];
   out_2432780480652285733[9] = 0;
   out_2432780480652285733[10] = 0;
   out_2432780480652285733[11] = 0;
   out_2432780480652285733[12] = 1;
   out_2432780480652285733[13] = 0;
   out_2432780480652285733[14] = 0;
   out_2432780480652285733[15] = 1;
   out_2432780480652285733[16] = 0;
   out_2432780480652285733[17] = 0;
   out_2432780480652285733[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_2432780480652285733[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_2432780480652285733[20] = 0;
   out_2432780480652285733[21] = state[8];
   out_2432780480652285733[22] = 0;
   out_2432780480652285733[23] = -state[6];
   out_2432780480652285733[24] = -state[5];
   out_2432780480652285733[25] = 0;
   out_2432780480652285733[26] = state[3];
   out_2432780480652285733[27] = 0;
   out_2432780480652285733[28] = 0;
   out_2432780480652285733[29] = 0;
   out_2432780480652285733[30] = 0;
   out_2432780480652285733[31] = 1;
   out_2432780480652285733[32] = 0;
   out_2432780480652285733[33] = 0;
   out_2432780480652285733[34] = 1;
   out_2432780480652285733[35] = 0;
   out_2432780480652285733[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_2432780480652285733[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_2432780480652285733[38] = 0;
   out_2432780480652285733[39] = -state[7];
   out_2432780480652285733[40] = state[6];
   out_2432780480652285733[41] = 0;
   out_2432780480652285733[42] = state[4];
   out_2432780480652285733[43] = -state[3];
   out_2432780480652285733[44] = 0;
   out_2432780480652285733[45] = 0;
   out_2432780480652285733[46] = 0;
   out_2432780480652285733[47] = 0;
   out_2432780480652285733[48] = 0;
   out_2432780480652285733[49] = 0;
   out_2432780480652285733[50] = 1;
   out_2432780480652285733[51] = 0;
   out_2432780480652285733[52] = 0;
   out_2432780480652285733[53] = 1;
}
void h_13(double *state, double *unused, double *out_5922169553294439551) {
   out_5922169553294439551[0] = state[3];
   out_5922169553294439551[1] = state[4];
   out_5922169553294439551[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2291681104729518876) {
   out_2291681104729518876[0] = 0;
   out_2291681104729518876[1] = 0;
   out_2291681104729518876[2] = 0;
   out_2291681104729518876[3] = 1;
   out_2291681104729518876[4] = 0;
   out_2291681104729518876[5] = 0;
   out_2291681104729518876[6] = 0;
   out_2291681104729518876[7] = 0;
   out_2291681104729518876[8] = 0;
   out_2291681104729518876[9] = 0;
   out_2291681104729518876[10] = 0;
   out_2291681104729518876[11] = 0;
   out_2291681104729518876[12] = 0;
   out_2291681104729518876[13] = 0;
   out_2291681104729518876[14] = 0;
   out_2291681104729518876[15] = 0;
   out_2291681104729518876[16] = 0;
   out_2291681104729518876[17] = 0;
   out_2291681104729518876[18] = 0;
   out_2291681104729518876[19] = 0;
   out_2291681104729518876[20] = 0;
   out_2291681104729518876[21] = 0;
   out_2291681104729518876[22] = 1;
   out_2291681104729518876[23] = 0;
   out_2291681104729518876[24] = 0;
   out_2291681104729518876[25] = 0;
   out_2291681104729518876[26] = 0;
   out_2291681104729518876[27] = 0;
   out_2291681104729518876[28] = 0;
   out_2291681104729518876[29] = 0;
   out_2291681104729518876[30] = 0;
   out_2291681104729518876[31] = 0;
   out_2291681104729518876[32] = 0;
   out_2291681104729518876[33] = 0;
   out_2291681104729518876[34] = 0;
   out_2291681104729518876[35] = 0;
   out_2291681104729518876[36] = 0;
   out_2291681104729518876[37] = 0;
   out_2291681104729518876[38] = 0;
   out_2291681104729518876[39] = 0;
   out_2291681104729518876[40] = 0;
   out_2291681104729518876[41] = 1;
   out_2291681104729518876[42] = 0;
   out_2291681104729518876[43] = 0;
   out_2291681104729518876[44] = 0;
   out_2291681104729518876[45] = 0;
   out_2291681104729518876[46] = 0;
   out_2291681104729518876[47] = 0;
   out_2291681104729518876[48] = 0;
   out_2291681104729518876[49] = 0;
   out_2291681104729518876[50] = 0;
   out_2291681104729518876[51] = 0;
   out_2291681104729518876[52] = 0;
   out_2291681104729518876[53] = 0;
}
void h_14(double *state, double *unused, double *out_2831406952902641910) {
   out_2831406952902641910[0] = state[6];
   out_2831406952902641910[1] = state[7];
   out_2831406952902641910[2] = state[8];
}
void H_14(double *state, double *unused, double *out_3042648135736670604) {
   out_3042648135736670604[0] = 0;
   out_3042648135736670604[1] = 0;
   out_3042648135736670604[2] = 0;
   out_3042648135736670604[3] = 0;
   out_3042648135736670604[4] = 0;
   out_3042648135736670604[5] = 0;
   out_3042648135736670604[6] = 1;
   out_3042648135736670604[7] = 0;
   out_3042648135736670604[8] = 0;
   out_3042648135736670604[9] = 0;
   out_3042648135736670604[10] = 0;
   out_3042648135736670604[11] = 0;
   out_3042648135736670604[12] = 0;
   out_3042648135736670604[13] = 0;
   out_3042648135736670604[14] = 0;
   out_3042648135736670604[15] = 0;
   out_3042648135736670604[16] = 0;
   out_3042648135736670604[17] = 0;
   out_3042648135736670604[18] = 0;
   out_3042648135736670604[19] = 0;
   out_3042648135736670604[20] = 0;
   out_3042648135736670604[21] = 0;
   out_3042648135736670604[22] = 0;
   out_3042648135736670604[23] = 0;
   out_3042648135736670604[24] = 0;
   out_3042648135736670604[25] = 1;
   out_3042648135736670604[26] = 0;
   out_3042648135736670604[27] = 0;
   out_3042648135736670604[28] = 0;
   out_3042648135736670604[29] = 0;
   out_3042648135736670604[30] = 0;
   out_3042648135736670604[31] = 0;
   out_3042648135736670604[32] = 0;
   out_3042648135736670604[33] = 0;
   out_3042648135736670604[34] = 0;
   out_3042648135736670604[35] = 0;
   out_3042648135736670604[36] = 0;
   out_3042648135736670604[37] = 0;
   out_3042648135736670604[38] = 0;
   out_3042648135736670604[39] = 0;
   out_3042648135736670604[40] = 0;
   out_3042648135736670604[41] = 0;
   out_3042648135736670604[42] = 0;
   out_3042648135736670604[43] = 0;
   out_3042648135736670604[44] = 1;
   out_3042648135736670604[45] = 0;
   out_3042648135736670604[46] = 0;
   out_3042648135736670604[47] = 0;
   out_3042648135736670604[48] = 0;
   out_3042648135736670604[49] = 0;
   out_3042648135736670604[50] = 0;
   out_3042648135736670604[51] = 0;
   out_3042648135736670604[52] = 0;
   out_3042648135736670604[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_4190350983600540570) {
  err_fun(nom_x, delta_x, out_4190350983600540570);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6497364289119633418) {
  inv_err_fun(nom_x, true_x, out_6497364289119633418);
}
void pose_H_mod_fun(double *state, double *out_4927489086149144796) {
  H_mod_fun(state, out_4927489086149144796);
}
void pose_f_fun(double *state, double dt, double *out_5720185918353944946) {
  f_fun(state,  dt, out_5720185918353944946);
}
void pose_F_fun(double *state, double dt, double *out_9170706574201723667) {
  F_fun(state,  dt, out_9170706574201723667);
}
void pose_h_4(double *state, double *unused, double *out_9123423313710806344) {
  h_4(state, unused, out_9123423313710806344);
}
void pose_H_4(double *state, double *unused, double *out_920592720602813925) {
  H_4(state, unused, out_920592720602813925);
}
void pose_h_10(double *state, double *unused, double *out_2741809755357044354) {
  h_10(state, unused, out_2741809755357044354);
}
void pose_H_10(double *state, double *unused, double *out_2432780480652285733) {
  H_10(state, unused, out_2432780480652285733);
}
void pose_h_13(double *state, double *unused, double *out_5922169553294439551) {
  h_13(state, unused, out_5922169553294439551);
}
void pose_H_13(double *state, double *unused, double *out_2291681104729518876) {
  H_13(state, unused, out_2291681104729518876);
}
void pose_h_14(double *state, double *unused, double *out_2831406952902641910) {
  h_14(state, unused, out_2831406952902641910);
}
void pose_H_14(double *state, double *unused, double *out_3042648135736670604) {
  H_14(state, unused, out_3042648135736670604);
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
