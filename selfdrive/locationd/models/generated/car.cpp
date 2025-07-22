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
void err_fun(double *nom_x, double *delta_x, double *out_4896524115569919767) {
   out_4896524115569919767[0] = delta_x[0] + nom_x[0];
   out_4896524115569919767[1] = delta_x[1] + nom_x[1];
   out_4896524115569919767[2] = delta_x[2] + nom_x[2];
   out_4896524115569919767[3] = delta_x[3] + nom_x[3];
   out_4896524115569919767[4] = delta_x[4] + nom_x[4];
   out_4896524115569919767[5] = delta_x[5] + nom_x[5];
   out_4896524115569919767[6] = delta_x[6] + nom_x[6];
   out_4896524115569919767[7] = delta_x[7] + nom_x[7];
   out_4896524115569919767[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8279430225237684735) {
   out_8279430225237684735[0] = -nom_x[0] + true_x[0];
   out_8279430225237684735[1] = -nom_x[1] + true_x[1];
   out_8279430225237684735[2] = -nom_x[2] + true_x[2];
   out_8279430225237684735[3] = -nom_x[3] + true_x[3];
   out_8279430225237684735[4] = -nom_x[4] + true_x[4];
   out_8279430225237684735[5] = -nom_x[5] + true_x[5];
   out_8279430225237684735[6] = -nom_x[6] + true_x[6];
   out_8279430225237684735[7] = -nom_x[7] + true_x[7];
   out_8279430225237684735[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5979659898168547992) {
   out_5979659898168547992[0] = 1.0;
   out_5979659898168547992[1] = 0.0;
   out_5979659898168547992[2] = 0.0;
   out_5979659898168547992[3] = 0.0;
   out_5979659898168547992[4] = 0.0;
   out_5979659898168547992[5] = 0.0;
   out_5979659898168547992[6] = 0.0;
   out_5979659898168547992[7] = 0.0;
   out_5979659898168547992[8] = 0.0;
   out_5979659898168547992[9] = 0.0;
   out_5979659898168547992[10] = 1.0;
   out_5979659898168547992[11] = 0.0;
   out_5979659898168547992[12] = 0.0;
   out_5979659898168547992[13] = 0.0;
   out_5979659898168547992[14] = 0.0;
   out_5979659898168547992[15] = 0.0;
   out_5979659898168547992[16] = 0.0;
   out_5979659898168547992[17] = 0.0;
   out_5979659898168547992[18] = 0.0;
   out_5979659898168547992[19] = 0.0;
   out_5979659898168547992[20] = 1.0;
   out_5979659898168547992[21] = 0.0;
   out_5979659898168547992[22] = 0.0;
   out_5979659898168547992[23] = 0.0;
   out_5979659898168547992[24] = 0.0;
   out_5979659898168547992[25] = 0.0;
   out_5979659898168547992[26] = 0.0;
   out_5979659898168547992[27] = 0.0;
   out_5979659898168547992[28] = 0.0;
   out_5979659898168547992[29] = 0.0;
   out_5979659898168547992[30] = 1.0;
   out_5979659898168547992[31] = 0.0;
   out_5979659898168547992[32] = 0.0;
   out_5979659898168547992[33] = 0.0;
   out_5979659898168547992[34] = 0.0;
   out_5979659898168547992[35] = 0.0;
   out_5979659898168547992[36] = 0.0;
   out_5979659898168547992[37] = 0.0;
   out_5979659898168547992[38] = 0.0;
   out_5979659898168547992[39] = 0.0;
   out_5979659898168547992[40] = 1.0;
   out_5979659898168547992[41] = 0.0;
   out_5979659898168547992[42] = 0.0;
   out_5979659898168547992[43] = 0.0;
   out_5979659898168547992[44] = 0.0;
   out_5979659898168547992[45] = 0.0;
   out_5979659898168547992[46] = 0.0;
   out_5979659898168547992[47] = 0.0;
   out_5979659898168547992[48] = 0.0;
   out_5979659898168547992[49] = 0.0;
   out_5979659898168547992[50] = 1.0;
   out_5979659898168547992[51] = 0.0;
   out_5979659898168547992[52] = 0.0;
   out_5979659898168547992[53] = 0.0;
   out_5979659898168547992[54] = 0.0;
   out_5979659898168547992[55] = 0.0;
   out_5979659898168547992[56] = 0.0;
   out_5979659898168547992[57] = 0.0;
   out_5979659898168547992[58] = 0.0;
   out_5979659898168547992[59] = 0.0;
   out_5979659898168547992[60] = 1.0;
   out_5979659898168547992[61] = 0.0;
   out_5979659898168547992[62] = 0.0;
   out_5979659898168547992[63] = 0.0;
   out_5979659898168547992[64] = 0.0;
   out_5979659898168547992[65] = 0.0;
   out_5979659898168547992[66] = 0.0;
   out_5979659898168547992[67] = 0.0;
   out_5979659898168547992[68] = 0.0;
   out_5979659898168547992[69] = 0.0;
   out_5979659898168547992[70] = 1.0;
   out_5979659898168547992[71] = 0.0;
   out_5979659898168547992[72] = 0.0;
   out_5979659898168547992[73] = 0.0;
   out_5979659898168547992[74] = 0.0;
   out_5979659898168547992[75] = 0.0;
   out_5979659898168547992[76] = 0.0;
   out_5979659898168547992[77] = 0.0;
   out_5979659898168547992[78] = 0.0;
   out_5979659898168547992[79] = 0.0;
   out_5979659898168547992[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8470978137202521173) {
   out_8470978137202521173[0] = state[0];
   out_8470978137202521173[1] = state[1];
   out_8470978137202521173[2] = state[2];
   out_8470978137202521173[3] = state[3];
   out_8470978137202521173[4] = state[4];
   out_8470978137202521173[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8470978137202521173[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8470978137202521173[7] = state[7];
   out_8470978137202521173[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6999648101104180157) {
   out_6999648101104180157[0] = 1;
   out_6999648101104180157[1] = 0;
   out_6999648101104180157[2] = 0;
   out_6999648101104180157[3] = 0;
   out_6999648101104180157[4] = 0;
   out_6999648101104180157[5] = 0;
   out_6999648101104180157[6] = 0;
   out_6999648101104180157[7] = 0;
   out_6999648101104180157[8] = 0;
   out_6999648101104180157[9] = 0;
   out_6999648101104180157[10] = 1;
   out_6999648101104180157[11] = 0;
   out_6999648101104180157[12] = 0;
   out_6999648101104180157[13] = 0;
   out_6999648101104180157[14] = 0;
   out_6999648101104180157[15] = 0;
   out_6999648101104180157[16] = 0;
   out_6999648101104180157[17] = 0;
   out_6999648101104180157[18] = 0;
   out_6999648101104180157[19] = 0;
   out_6999648101104180157[20] = 1;
   out_6999648101104180157[21] = 0;
   out_6999648101104180157[22] = 0;
   out_6999648101104180157[23] = 0;
   out_6999648101104180157[24] = 0;
   out_6999648101104180157[25] = 0;
   out_6999648101104180157[26] = 0;
   out_6999648101104180157[27] = 0;
   out_6999648101104180157[28] = 0;
   out_6999648101104180157[29] = 0;
   out_6999648101104180157[30] = 1;
   out_6999648101104180157[31] = 0;
   out_6999648101104180157[32] = 0;
   out_6999648101104180157[33] = 0;
   out_6999648101104180157[34] = 0;
   out_6999648101104180157[35] = 0;
   out_6999648101104180157[36] = 0;
   out_6999648101104180157[37] = 0;
   out_6999648101104180157[38] = 0;
   out_6999648101104180157[39] = 0;
   out_6999648101104180157[40] = 1;
   out_6999648101104180157[41] = 0;
   out_6999648101104180157[42] = 0;
   out_6999648101104180157[43] = 0;
   out_6999648101104180157[44] = 0;
   out_6999648101104180157[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6999648101104180157[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6999648101104180157[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6999648101104180157[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6999648101104180157[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6999648101104180157[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6999648101104180157[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6999648101104180157[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6999648101104180157[53] = -9.8000000000000007*dt;
   out_6999648101104180157[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6999648101104180157[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6999648101104180157[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6999648101104180157[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6999648101104180157[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6999648101104180157[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6999648101104180157[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6999648101104180157[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6999648101104180157[62] = 0;
   out_6999648101104180157[63] = 0;
   out_6999648101104180157[64] = 0;
   out_6999648101104180157[65] = 0;
   out_6999648101104180157[66] = 0;
   out_6999648101104180157[67] = 0;
   out_6999648101104180157[68] = 0;
   out_6999648101104180157[69] = 0;
   out_6999648101104180157[70] = 1;
   out_6999648101104180157[71] = 0;
   out_6999648101104180157[72] = 0;
   out_6999648101104180157[73] = 0;
   out_6999648101104180157[74] = 0;
   out_6999648101104180157[75] = 0;
   out_6999648101104180157[76] = 0;
   out_6999648101104180157[77] = 0;
   out_6999648101104180157[78] = 0;
   out_6999648101104180157[79] = 0;
   out_6999648101104180157[80] = 1;
}
void h_25(double *state, double *unused, double *out_2637657056612919151) {
   out_2637657056612919151[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1055254409831246330) {
   out_1055254409831246330[0] = 0;
   out_1055254409831246330[1] = 0;
   out_1055254409831246330[2] = 0;
   out_1055254409831246330[3] = 0;
   out_1055254409831246330[4] = 0;
   out_1055254409831246330[5] = 0;
   out_1055254409831246330[6] = 1;
   out_1055254409831246330[7] = 0;
   out_1055254409831246330[8] = 0;
}
void h_24(double *state, double *unused, double *out_1354494922998267052) {
   out_1354494922998267052[0] = state[4];
   out_1354494922998267052[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2502671188971076806) {
   out_2502671188971076806[0] = 0;
   out_2502671188971076806[1] = 0;
   out_2502671188971076806[2] = 0;
   out_2502671188971076806[3] = 0;
   out_2502671188971076806[4] = 1;
   out_2502671188971076806[5] = 0;
   out_2502671188971076806[6] = 0;
   out_2502671188971076806[7] = 0;
   out_2502671188971076806[8] = 0;
   out_2502671188971076806[9] = 0;
   out_2502671188971076806[10] = 0;
   out_2502671188971076806[11] = 0;
   out_2502671188971076806[12] = 0;
   out_2502671188971076806[13] = 0;
   out_2502671188971076806[14] = 1;
   out_2502671188971076806[15] = 0;
   out_2502671188971076806[16] = 0;
   out_2502671188971076806[17] = 0;
}
void h_30(double *state, double *unused, double *out_4088288449241802318) {
   out_4088288449241802318[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5861435931660370425) {
   out_5861435931660370425[0] = 0;
   out_5861435931660370425[1] = 0;
   out_5861435931660370425[2] = 0;
   out_5861435931660370425[3] = 0;
   out_5861435931660370425[4] = 1;
   out_5861435931660370425[5] = 0;
   out_5861435931660370425[6] = 0;
   out_5861435931660370425[7] = 0;
   out_5861435931660370425[8] = 0;
}
void h_26(double *state, double *unused, double *out_1669696464751935638) {
   out_1669696464751935638[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4796757728705302554) {
   out_4796757728705302554[0] = 0;
   out_4796757728705302554[1] = 0;
   out_4796757728705302554[2] = 0;
   out_4796757728705302554[3] = 0;
   out_4796757728705302554[4] = 0;
   out_4796757728705302554[5] = 0;
   out_4796757728705302554[6] = 0;
   out_4796757728705302554[7] = 1;
   out_4796757728705302554[8] = 0;
}
void h_27(double *state, double *unused, double *out_5948048967848081832) {
   out_5948048967848081832[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3686672619859945514) {
   out_3686672619859945514[0] = 0;
   out_3686672619859945514[1] = 0;
   out_3686672619859945514[2] = 0;
   out_3686672619859945514[3] = 1;
   out_3686672619859945514[4] = 0;
   out_3686672619859945514[5] = 0;
   out_3686672619859945514[6] = 0;
   out_3686672619859945514[7] = 0;
   out_3686672619859945514[8] = 0;
}
void h_29(double *state, double *unused, double *out_913384818419043327) {
   out_913384818419043327[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6371667275974762609) {
   out_6371667275974762609[0] = 0;
   out_6371667275974762609[1] = 1;
   out_6371667275974762609[2] = 0;
   out_6371667275974762609[3] = 0;
   out_6371667275974762609[4] = 0;
   out_6371667275974762609[5] = 0;
   out_6371667275974762609[6] = 0;
   out_6371667275974762609[7] = 0;
   out_6371667275974762609[8] = 0;
}
void h_28(double *state, double *unused, double *out_6617952960958914271) {
   out_6617952960958914271[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3109089124079136093) {
   out_3109089124079136093[0] = 1;
   out_3109089124079136093[1] = 0;
   out_3109089124079136093[2] = 0;
   out_3109089124079136093[3] = 0;
   out_3109089124079136093[4] = 0;
   out_3109089124079136093[5] = 0;
   out_3109089124079136093[6] = 0;
   out_3109089124079136093[7] = 0;
   out_3109089124079136093[8] = 0;
}
void h_31(double *state, double *unused, double *out_4133178169737431785) {
   out_4133178169737431785[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1024608447954285902) {
   out_1024608447954285902[0] = 0;
   out_1024608447954285902[1] = 0;
   out_1024608447954285902[2] = 0;
   out_1024608447954285902[3] = 0;
   out_1024608447954285902[4] = 0;
   out_1024608447954285902[5] = 0;
   out_1024608447954285902[6] = 0;
   out_1024608447954285902[7] = 0;
   out_1024608447954285902[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4896524115569919767) {
  err_fun(nom_x, delta_x, out_4896524115569919767);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8279430225237684735) {
  inv_err_fun(nom_x, true_x, out_8279430225237684735);
}
void car_H_mod_fun(double *state, double *out_5979659898168547992) {
  H_mod_fun(state, out_5979659898168547992);
}
void car_f_fun(double *state, double dt, double *out_8470978137202521173) {
  f_fun(state,  dt, out_8470978137202521173);
}
void car_F_fun(double *state, double dt, double *out_6999648101104180157) {
  F_fun(state,  dt, out_6999648101104180157);
}
void car_h_25(double *state, double *unused, double *out_2637657056612919151) {
  h_25(state, unused, out_2637657056612919151);
}
void car_H_25(double *state, double *unused, double *out_1055254409831246330) {
  H_25(state, unused, out_1055254409831246330);
}
void car_h_24(double *state, double *unused, double *out_1354494922998267052) {
  h_24(state, unused, out_1354494922998267052);
}
void car_H_24(double *state, double *unused, double *out_2502671188971076806) {
  H_24(state, unused, out_2502671188971076806);
}
void car_h_30(double *state, double *unused, double *out_4088288449241802318) {
  h_30(state, unused, out_4088288449241802318);
}
void car_H_30(double *state, double *unused, double *out_5861435931660370425) {
  H_30(state, unused, out_5861435931660370425);
}
void car_h_26(double *state, double *unused, double *out_1669696464751935638) {
  h_26(state, unused, out_1669696464751935638);
}
void car_H_26(double *state, double *unused, double *out_4796757728705302554) {
  H_26(state, unused, out_4796757728705302554);
}
void car_h_27(double *state, double *unused, double *out_5948048967848081832) {
  h_27(state, unused, out_5948048967848081832);
}
void car_H_27(double *state, double *unused, double *out_3686672619859945514) {
  H_27(state, unused, out_3686672619859945514);
}
void car_h_29(double *state, double *unused, double *out_913384818419043327) {
  h_29(state, unused, out_913384818419043327);
}
void car_H_29(double *state, double *unused, double *out_6371667275974762609) {
  H_29(state, unused, out_6371667275974762609);
}
void car_h_28(double *state, double *unused, double *out_6617952960958914271) {
  h_28(state, unused, out_6617952960958914271);
}
void car_H_28(double *state, double *unused, double *out_3109089124079136093) {
  H_28(state, unused, out_3109089124079136093);
}
void car_h_31(double *state, double *unused, double *out_4133178169737431785) {
  h_31(state, unused, out_4133178169737431785);
}
void car_H_31(double *state, double *unused, double *out_1024608447954285902) {
  H_31(state, unused, out_1024608447954285902);
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
