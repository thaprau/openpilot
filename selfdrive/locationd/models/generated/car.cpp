#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_418323971458922723) {
   out_418323971458922723[0] = delta_x[0] + nom_x[0];
   out_418323971458922723[1] = delta_x[1] + nom_x[1];
   out_418323971458922723[2] = delta_x[2] + nom_x[2];
   out_418323971458922723[3] = delta_x[3] + nom_x[3];
   out_418323971458922723[4] = delta_x[4] + nom_x[4];
   out_418323971458922723[5] = delta_x[5] + nom_x[5];
   out_418323971458922723[6] = delta_x[6] + nom_x[6];
   out_418323971458922723[7] = delta_x[7] + nom_x[7];
   out_418323971458922723[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4558772615268068502) {
   out_4558772615268068502[0] = -nom_x[0] + true_x[0];
   out_4558772615268068502[1] = -nom_x[1] + true_x[1];
   out_4558772615268068502[2] = -nom_x[2] + true_x[2];
   out_4558772615268068502[3] = -nom_x[3] + true_x[3];
   out_4558772615268068502[4] = -nom_x[4] + true_x[4];
   out_4558772615268068502[5] = -nom_x[5] + true_x[5];
   out_4558772615268068502[6] = -nom_x[6] + true_x[6];
   out_4558772615268068502[7] = -nom_x[7] + true_x[7];
   out_4558772615268068502[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6987128163890705878) {
   out_6987128163890705878[0] = 1.0;
   out_6987128163890705878[1] = 0.0;
   out_6987128163890705878[2] = 0.0;
   out_6987128163890705878[3] = 0.0;
   out_6987128163890705878[4] = 0.0;
   out_6987128163890705878[5] = 0.0;
   out_6987128163890705878[6] = 0.0;
   out_6987128163890705878[7] = 0.0;
   out_6987128163890705878[8] = 0.0;
   out_6987128163890705878[9] = 0.0;
   out_6987128163890705878[10] = 1.0;
   out_6987128163890705878[11] = 0.0;
   out_6987128163890705878[12] = 0.0;
   out_6987128163890705878[13] = 0.0;
   out_6987128163890705878[14] = 0.0;
   out_6987128163890705878[15] = 0.0;
   out_6987128163890705878[16] = 0.0;
   out_6987128163890705878[17] = 0.0;
   out_6987128163890705878[18] = 0.0;
   out_6987128163890705878[19] = 0.0;
   out_6987128163890705878[20] = 1.0;
   out_6987128163890705878[21] = 0.0;
   out_6987128163890705878[22] = 0.0;
   out_6987128163890705878[23] = 0.0;
   out_6987128163890705878[24] = 0.0;
   out_6987128163890705878[25] = 0.0;
   out_6987128163890705878[26] = 0.0;
   out_6987128163890705878[27] = 0.0;
   out_6987128163890705878[28] = 0.0;
   out_6987128163890705878[29] = 0.0;
   out_6987128163890705878[30] = 1.0;
   out_6987128163890705878[31] = 0.0;
   out_6987128163890705878[32] = 0.0;
   out_6987128163890705878[33] = 0.0;
   out_6987128163890705878[34] = 0.0;
   out_6987128163890705878[35] = 0.0;
   out_6987128163890705878[36] = 0.0;
   out_6987128163890705878[37] = 0.0;
   out_6987128163890705878[38] = 0.0;
   out_6987128163890705878[39] = 0.0;
   out_6987128163890705878[40] = 1.0;
   out_6987128163890705878[41] = 0.0;
   out_6987128163890705878[42] = 0.0;
   out_6987128163890705878[43] = 0.0;
   out_6987128163890705878[44] = 0.0;
   out_6987128163890705878[45] = 0.0;
   out_6987128163890705878[46] = 0.0;
   out_6987128163890705878[47] = 0.0;
   out_6987128163890705878[48] = 0.0;
   out_6987128163890705878[49] = 0.0;
   out_6987128163890705878[50] = 1.0;
   out_6987128163890705878[51] = 0.0;
   out_6987128163890705878[52] = 0.0;
   out_6987128163890705878[53] = 0.0;
   out_6987128163890705878[54] = 0.0;
   out_6987128163890705878[55] = 0.0;
   out_6987128163890705878[56] = 0.0;
   out_6987128163890705878[57] = 0.0;
   out_6987128163890705878[58] = 0.0;
   out_6987128163890705878[59] = 0.0;
   out_6987128163890705878[60] = 1.0;
   out_6987128163890705878[61] = 0.0;
   out_6987128163890705878[62] = 0.0;
   out_6987128163890705878[63] = 0.0;
   out_6987128163890705878[64] = 0.0;
   out_6987128163890705878[65] = 0.0;
   out_6987128163890705878[66] = 0.0;
   out_6987128163890705878[67] = 0.0;
   out_6987128163890705878[68] = 0.0;
   out_6987128163890705878[69] = 0.0;
   out_6987128163890705878[70] = 1.0;
   out_6987128163890705878[71] = 0.0;
   out_6987128163890705878[72] = 0.0;
   out_6987128163890705878[73] = 0.0;
   out_6987128163890705878[74] = 0.0;
   out_6987128163890705878[75] = 0.0;
   out_6987128163890705878[76] = 0.0;
   out_6987128163890705878[77] = 0.0;
   out_6987128163890705878[78] = 0.0;
   out_6987128163890705878[79] = 0.0;
   out_6987128163890705878[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_642478848160592513) {
   out_642478848160592513[0] = state[0];
   out_642478848160592513[1] = state[1];
   out_642478848160592513[2] = state[2];
   out_642478848160592513[3] = state[3];
   out_642478848160592513[4] = state[4];
   out_642478848160592513[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_642478848160592513[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_642478848160592513[7] = state[7];
   out_642478848160592513[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8728087220090014278) {
   out_8728087220090014278[0] = 1;
   out_8728087220090014278[1] = 0;
   out_8728087220090014278[2] = 0;
   out_8728087220090014278[3] = 0;
   out_8728087220090014278[4] = 0;
   out_8728087220090014278[5] = 0;
   out_8728087220090014278[6] = 0;
   out_8728087220090014278[7] = 0;
   out_8728087220090014278[8] = 0;
   out_8728087220090014278[9] = 0;
   out_8728087220090014278[10] = 1;
   out_8728087220090014278[11] = 0;
   out_8728087220090014278[12] = 0;
   out_8728087220090014278[13] = 0;
   out_8728087220090014278[14] = 0;
   out_8728087220090014278[15] = 0;
   out_8728087220090014278[16] = 0;
   out_8728087220090014278[17] = 0;
   out_8728087220090014278[18] = 0;
   out_8728087220090014278[19] = 0;
   out_8728087220090014278[20] = 1;
   out_8728087220090014278[21] = 0;
   out_8728087220090014278[22] = 0;
   out_8728087220090014278[23] = 0;
   out_8728087220090014278[24] = 0;
   out_8728087220090014278[25] = 0;
   out_8728087220090014278[26] = 0;
   out_8728087220090014278[27] = 0;
   out_8728087220090014278[28] = 0;
   out_8728087220090014278[29] = 0;
   out_8728087220090014278[30] = 1;
   out_8728087220090014278[31] = 0;
   out_8728087220090014278[32] = 0;
   out_8728087220090014278[33] = 0;
   out_8728087220090014278[34] = 0;
   out_8728087220090014278[35] = 0;
   out_8728087220090014278[36] = 0;
   out_8728087220090014278[37] = 0;
   out_8728087220090014278[38] = 0;
   out_8728087220090014278[39] = 0;
   out_8728087220090014278[40] = 1;
   out_8728087220090014278[41] = 0;
   out_8728087220090014278[42] = 0;
   out_8728087220090014278[43] = 0;
   out_8728087220090014278[44] = 0;
   out_8728087220090014278[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8728087220090014278[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8728087220090014278[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8728087220090014278[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8728087220090014278[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8728087220090014278[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8728087220090014278[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8728087220090014278[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8728087220090014278[53] = -9.8100000000000005*dt;
   out_8728087220090014278[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8728087220090014278[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8728087220090014278[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8728087220090014278[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8728087220090014278[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8728087220090014278[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8728087220090014278[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8728087220090014278[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8728087220090014278[62] = 0;
   out_8728087220090014278[63] = 0;
   out_8728087220090014278[64] = 0;
   out_8728087220090014278[65] = 0;
   out_8728087220090014278[66] = 0;
   out_8728087220090014278[67] = 0;
   out_8728087220090014278[68] = 0;
   out_8728087220090014278[69] = 0;
   out_8728087220090014278[70] = 1;
   out_8728087220090014278[71] = 0;
   out_8728087220090014278[72] = 0;
   out_8728087220090014278[73] = 0;
   out_8728087220090014278[74] = 0;
   out_8728087220090014278[75] = 0;
   out_8728087220090014278[76] = 0;
   out_8728087220090014278[77] = 0;
   out_8728087220090014278[78] = 0;
   out_8728087220090014278[79] = 0;
   out_8728087220090014278[80] = 1;
}
void h_25(double *state, double *unused, double *out_8060502601363644427) {
   out_8060502601363644427[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2192297320962836769) {
   out_2192297320962836769[0] = 0;
   out_2192297320962836769[1] = 0;
   out_2192297320962836769[2] = 0;
   out_2192297320962836769[3] = 0;
   out_2192297320962836769[4] = 0;
   out_2192297320962836769[5] = 0;
   out_2192297320962836769[6] = 1;
   out_2192297320962836769[7] = 0;
   out_2192297320962836769[8] = 0;
}
void h_24(double *state, double *unused, double *out_3013396144018423090) {
   out_3013396144018423090[0] = state[4];
   out_3013396144018423090[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3605325569001245606) {
   out_3605325569001245606[0] = 0;
   out_3605325569001245606[1] = 0;
   out_3605325569001245606[2] = 0;
   out_3605325569001245606[3] = 0;
   out_3605325569001245606[4] = 1;
   out_3605325569001245606[5] = 0;
   out_3605325569001245606[6] = 0;
   out_3605325569001245606[7] = 0;
   out_3605325569001245606[8] = 0;
   out_3605325569001245606[9] = 0;
   out_3605325569001245606[10] = 0;
   out_3605325569001245606[11] = 0;
   out_3605325569001245606[12] = 0;
   out_3605325569001245606[13] = 0;
   out_3605325569001245606[14] = 1;
   out_3605325569001245606[15] = 0;
   out_3605325569001245606[16] = 0;
   out_3605325569001245606[17] = 0;
}
void h_30(double *state, double *unused, double *out_3962199303084272563) {
   out_3962199303084272563[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4710630279470085396) {
   out_4710630279470085396[0] = 0;
   out_4710630279470085396[1] = 0;
   out_4710630279470085396[2] = 0;
   out_4710630279470085396[3] = 0;
   out_4710630279470085396[4] = 1;
   out_4710630279470085396[5] = 0;
   out_4710630279470085396[6] = 0;
   out_4710630279470085396[7] = 0;
   out_4710630279470085396[8] = 0;
}
void h_26(double *state, double *unused, double *out_5058344109356203879) {
   out_5058344109356203879[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1549205997911219455) {
   out_1549205997911219455[0] = 0;
   out_1549205997911219455[1] = 0;
   out_1549205997911219455[2] = 0;
   out_1549205997911219455[3] = 0;
   out_1549205997911219455[4] = 0;
   out_1549205997911219455[5] = 0;
   out_1549205997911219455[6] = 0;
   out_1549205997911219455[7] = 1;
   out_1549205997911219455[8] = 0;
}
void h_27(double *state, double *unused, double *out_5166933345374113634) {
   out_5166933345374113634[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2535866967669660485) {
   out_2535866967669660485[0] = 0;
   out_2535866967669660485[1] = 0;
   out_2535866967669660485[2] = 0;
   out_2535866967669660485[3] = 1;
   out_2535866967669660485[4] = 0;
   out_2535866967669660485[5] = 0;
   out_2535866967669660485[6] = 0;
   out_2535866967669660485[7] = 0;
   out_2535866967669660485[8] = 0;
}
void h_29(double *state, double *unused, double *out_4565515187760750080) {
   out_4565515187760750080[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5220861623784477580) {
   out_5220861623784477580[0] = 0;
   out_5220861623784477580[1] = 1;
   out_5220861623784477580[2] = 0;
   out_5220861623784477580[3] = 0;
   out_5220861623784477580[4] = 0;
   out_5220861623784477580[5] = 0;
   out_5220861623784477580[6] = 0;
   out_5220861623784477580[7] = 0;
   out_5220861623784477580[8] = 0;
}
void h_28(double *state, double *unused, double *out_4080206697017649307) {
   out_4080206697017649307[0] = state[0];
}
void H_28(double *state, double *unused, double *out_138462606714947006) {
   out_138462606714947006[0] = 1;
   out_138462606714947006[1] = 0;
   out_138462606714947006[2] = 0;
   out_138462606714947006[3] = 0;
   out_138462606714947006[4] = 0;
   out_138462606714947006[5] = 0;
   out_138462606714947006[6] = 0;
   out_138462606714947006[7] = 0;
   out_138462606714947006[8] = 0;
}
void h_31(double *state, double *unused, double *out_7785308539079138538) {
   out_7785308539079138538[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2175414100144570931) {
   out_2175414100144570931[0] = 0;
   out_2175414100144570931[1] = 0;
   out_2175414100144570931[2] = 0;
   out_2175414100144570931[3] = 0;
   out_2175414100144570931[4] = 0;
   out_2175414100144570931[5] = 0;
   out_2175414100144570931[6] = 0;
   out_2175414100144570931[7] = 0;
   out_2175414100144570931[8] = 1;
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

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_418323971458922723) {
  err_fun(nom_x, delta_x, out_418323971458922723);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4558772615268068502) {
  inv_err_fun(nom_x, true_x, out_4558772615268068502);
}
void car_H_mod_fun(double *state, double *out_6987128163890705878) {
  H_mod_fun(state, out_6987128163890705878);
}
void car_f_fun(double *state, double dt, double *out_642478848160592513) {
  f_fun(state,  dt, out_642478848160592513);
}
void car_F_fun(double *state, double dt, double *out_8728087220090014278) {
  F_fun(state,  dt, out_8728087220090014278);
}
void car_h_25(double *state, double *unused, double *out_8060502601363644427) {
  h_25(state, unused, out_8060502601363644427);
}
void car_H_25(double *state, double *unused, double *out_2192297320962836769) {
  H_25(state, unused, out_2192297320962836769);
}
void car_h_24(double *state, double *unused, double *out_3013396144018423090) {
  h_24(state, unused, out_3013396144018423090);
}
void car_H_24(double *state, double *unused, double *out_3605325569001245606) {
  H_24(state, unused, out_3605325569001245606);
}
void car_h_30(double *state, double *unused, double *out_3962199303084272563) {
  h_30(state, unused, out_3962199303084272563);
}
void car_H_30(double *state, double *unused, double *out_4710630279470085396) {
  H_30(state, unused, out_4710630279470085396);
}
void car_h_26(double *state, double *unused, double *out_5058344109356203879) {
  h_26(state, unused, out_5058344109356203879);
}
void car_H_26(double *state, double *unused, double *out_1549205997911219455) {
  H_26(state, unused, out_1549205997911219455);
}
void car_h_27(double *state, double *unused, double *out_5166933345374113634) {
  h_27(state, unused, out_5166933345374113634);
}
void car_H_27(double *state, double *unused, double *out_2535866967669660485) {
  H_27(state, unused, out_2535866967669660485);
}
void car_h_29(double *state, double *unused, double *out_4565515187760750080) {
  h_29(state, unused, out_4565515187760750080);
}
void car_H_29(double *state, double *unused, double *out_5220861623784477580) {
  H_29(state, unused, out_5220861623784477580);
}
void car_h_28(double *state, double *unused, double *out_4080206697017649307) {
  h_28(state, unused, out_4080206697017649307);
}
void car_H_28(double *state, double *unused, double *out_138462606714947006) {
  H_28(state, unused, out_138462606714947006);
}
void car_h_31(double *state, double *unused, double *out_7785308539079138538) {
  h_31(state, unused, out_7785308539079138538);
}
void car_H_31(double *state, double *unused, double *out_2175414100144570931) {
  H_31(state, unused, out_2175414100144570931);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
