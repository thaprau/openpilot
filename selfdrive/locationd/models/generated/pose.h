#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_4220870942759577714);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6691487759945560833);
void pose_H_mod_fun(double *state, double *out_5674745270205462321);
void pose_f_fun(double *state, double dt, double *out_8950409172442745226);
void pose_F_fun(double *state, double dt, double *out_7351665863096111010);
void pose_h_4(double *state, double *unused, double *out_3301009148839050449);
void pose_H_4(double *state, double *unused, double *out_6872692752088360425);
void pose_h_10(double *state, double *unused, double *out_977489843187526232);
void pose_H_10(double *state, double *unused, double *out_6075288928525757853);
void pose_h_13(double *state, double *unused, double *out_8626245788850028663);
void pose_H_13(double *state, double *unused, double *out_3038937288785836401);
void pose_h_14(double *state, double *unused, double *out_4634144118535572398);
void pose_H_14(double *state, double *unused, double *out_3789904319792988129);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}