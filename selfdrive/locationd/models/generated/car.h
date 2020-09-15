/******************************************************************************
 *                       Code generated with sympy 1.4                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_727470535757866170);
void inv_err_fun(double *nom_x, double *true_x, double *out_7823608325048859542);
void H_mod_fun(double *state, double *out_8561215076071933433);
void f_fun(double *state, double dt, double *out_5854494543467114630);
void F_fun(double *state, double dt, double *out_7214143257728792534);
void h_25(double *state, double *unused, double *out_604571298292658417);
void H_25(double *state, double *unused, double *out_3286662799098636255);
void h_24(double *state, double *unused, double *out_5542032034878286667);
void H_24(double *state, double *unused, double *out_3810660555687201395);
void h_30(double *state, double *unused, double *out_4521560671013167731);
void H_30(double *state, double *unused, double *out_2965787034756330037);
void h_26(double *state, double *unused, double *out_8202605294476525334);
void H_26(double *state, double *unused, double *out_2359552449907263145);
void h_27(double *state, double *unused, double *out_7009077137081734482);
void H_27(double *state, double *unused, double *out_4843577531083537915);
void h_29(double *state, double *unused, double *out_3640713933842738224);
void H_29(double *state, double *unused, double *out_389076561956202671);
void h_28(double *state, double *unused, double *out_5364533368482937023);
void H_28(double *state, double *unused, double *out_7941601476428430007);
#define DIM 8
#define EDIM 8
#define MEDIM 8
typedef void (*Hfun)(double *, double *, double *);

void predict(double *x, double *P, double *Q, double dt);
const static double MAHA_THRESH_25 = 3.841459;
void update_25(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_24 = 5.991465;
void update_24(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_30 = 3.841459;
void update_30(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_26 = 3.841459;
void update_26(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_27 = 3.841459;
void update_27(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_29 = 3.841459;
void update_29(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_28 = 5.991465;
void update_28(double *, double *, double *, double *, double *);
void set_mass(double x);

void set_rotational_inertia(double x);

void set_center_to_front(double x);

void set_center_to_rear(double x);

void set_stiffness_front(double x);

void set_stiffness_rear(double x);
