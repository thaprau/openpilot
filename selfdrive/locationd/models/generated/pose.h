#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_4190350983600540570);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6497364289119633418);
void pose_H_mod_fun(double *state, double *out_4927489086149144796);
void pose_f_fun(double *state, double dt, double *out_5720185918353944946);
void pose_F_fun(double *state, double dt, double *out_9170706574201723667);
void pose_h_4(double *state, double *unused, double *out_9123423313710806344);
void pose_H_4(double *state, double *unused, double *out_920592720602813925);
void pose_h_10(double *state, double *unused, double *out_2741809755357044354);
void pose_H_10(double *state, double *unused, double *out_2432780480652285733);
void pose_h_13(double *state, double *unused, double *out_5922169553294439551);
void pose_H_13(double *state, double *unused, double *out_2291681104729518876);
void pose_h_14(double *state, double *unused, double *out_2831406952902641910);
void pose_H_14(double *state, double *unused, double *out_3042648135736670604);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}