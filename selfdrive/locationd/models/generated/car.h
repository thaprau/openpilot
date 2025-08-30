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
void car_err_fun(double *nom_x, double *delta_x, double *out_418323971458922723);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4558772615268068502);
void car_H_mod_fun(double *state, double *out_6987128163890705878);
void car_f_fun(double *state, double dt, double *out_642478848160592513);
void car_F_fun(double *state, double dt, double *out_8728087220090014278);
void car_h_25(double *state, double *unused, double *out_8060502601363644427);
void car_H_25(double *state, double *unused, double *out_2192297320962836769);
void car_h_24(double *state, double *unused, double *out_3013396144018423090);
void car_H_24(double *state, double *unused, double *out_3605325569001245606);
void car_h_30(double *state, double *unused, double *out_3962199303084272563);
void car_H_30(double *state, double *unused, double *out_4710630279470085396);
void car_h_26(double *state, double *unused, double *out_5058344109356203879);
void car_H_26(double *state, double *unused, double *out_1549205997911219455);
void car_h_27(double *state, double *unused, double *out_5166933345374113634);
void car_H_27(double *state, double *unused, double *out_2535866967669660485);
void car_h_29(double *state, double *unused, double *out_4565515187760750080);
void car_H_29(double *state, double *unused, double *out_5220861623784477580);
void car_h_28(double *state, double *unused, double *out_4080206697017649307);
void car_H_28(double *state, double *unused, double *out_138462606714947006);
void car_h_31(double *state, double *unused, double *out_7785308539079138538);
void car_H_31(double *state, double *unused, double *out_2175414100144570931);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}