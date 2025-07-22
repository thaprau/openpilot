#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_4896524115569919767);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8279430225237684735);
void car_H_mod_fun(double *state, double *out_5979659898168547992);
void car_f_fun(double *state, double dt, double *out_8470978137202521173);
void car_F_fun(double *state, double dt, double *out_6999648101104180157);
void car_h_25(double *state, double *unused, double *out_2637657056612919151);
void car_H_25(double *state, double *unused, double *out_1055254409831246330);
void car_h_24(double *state, double *unused, double *out_1354494922998267052);
void car_H_24(double *state, double *unused, double *out_2502671188971076806);
void car_h_30(double *state, double *unused, double *out_4088288449241802318);
void car_H_30(double *state, double *unused, double *out_5861435931660370425);
void car_h_26(double *state, double *unused, double *out_1669696464751935638);
void car_H_26(double *state, double *unused, double *out_4796757728705302554);
void car_h_27(double *state, double *unused, double *out_5948048967848081832);
void car_H_27(double *state, double *unused, double *out_3686672619859945514);
void car_h_29(double *state, double *unused, double *out_913384818419043327);
void car_H_29(double *state, double *unused, double *out_6371667275974762609);
void car_h_28(double *state, double *unused, double *out_6617952960958914271);
void car_H_28(double *state, double *unused, double *out_3109089124079136093);
void car_h_31(double *state, double *unused, double *out_4133178169737431785);
void car_H_31(double *state, double *unused, double *out_1024608447954285902);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}