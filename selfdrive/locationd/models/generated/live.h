/******************************************************************************
 *                       Code generated with sympy 1.4                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5294820582126360503);
void inv_err_fun(double *nom_x, double *true_x, double *out_8291686926427668403);
void H_mod_fun(double *state, double *out_2877994446572351358);
void f_fun(double *state, double dt, double *out_8392934663661768923);
void F_fun(double *state, double dt, double *out_5861153544503324366);
void h_3(double *state, double *unused, double *out_3868186964773160293);
void H_3(double *state, double *unused, double *out_5376435842204275973);
void h_4(double *state, double *unused, double *out_9116905882572664513);
void H_4(double *state, double *unused, double *out_8495510095846090169);
void h_9(double *state, double *unused, double *out_3448913356607650645);
void H_9(double *state, double *unused, double *out_7039253329680652696);
void h_10(double *state, double *unused, double *out_6706949316926160379);
void H_10(double *state, double *unused, double *out_1833084162599582565);
void h_12(double *state, double *unused, double *out_8423237941610249871);
void H_12(double *state, double *unused, double *out_3820450721007290520);
void h_31(double *state, double *unused, double *out_8690560997007505884);
void H_31(double *state, double *unused, double *out_8150915990810666832);
void h_32(double *state, double *unused, double *out_5056002201611583228);
void H_32(double *state, double *unused, double *out_5440093391983705918);
void h_13(double *state, double *unused, double *out_7442253765833324764);
void H_13(double *state, double *unused, double *out_4117750663455693000);
void h_14(double *state, double *unused, double *out_3448913356607650645);
void H_14(double *state, double *unused, double *out_7039253329680652696);
void h_19(double *state, double *unused, double *out_8212180038210347555);
void H_19(double *state, double *unused, double *out_9173260175408874980);
#define DIM 23
#define EDIM 22
#define MEDIM 22
typedef void (*Hfun)(double *, double *, double *);

void predict(double *x, double *P, double *Q, double dt);
const static double MAHA_THRESH_3 = 3.841459;
void update_3(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814728;
void update_4(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_9 = 7.814728;
void update_9(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_10 = 7.814728;
void update_10(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_12 = 7.814728;
void update_12(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_31 = 7.814728;
void update_31(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_32 = 9.487729;
void update_32(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_13 = 7.814728;
void update_13(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_14 = 7.814728;
void update_14(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_19 = 7.814728;
void update_19(double *, double *, double *, double *, double *);