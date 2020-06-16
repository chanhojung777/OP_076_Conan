/******************************************************************************
 *                       Code generated with sympy 1.4                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3096262396608016595);
void inv_err_fun(double *nom_x, double *true_x, double *out_6808884815189872555);
void H_mod_fun(double *state, double *out_8151182089197888992);
void f_fun(double *state, double dt, double *out_8080945692066930072);
void F_fun(double *state, double dt, double *out_7407822109033065220);
void h_25(double *state, double *unused, double *out_2684688280309935765);
void H_25(double *state, double *unused, double *out_8884442395634928840);
void h_24(double *state, double *unused, double *out_230005947588702794);
void H_24(double *state, double *unused, double *out_7275217207707909980);
void h_30(double *state, double *unused, double *out_4935230695321287415);
void H_30(double *state, double *unused, double *out_7262152278745433074);
void h_26(double *state, double *unused, double *out_2615980980938838772);
void H_26(double *state, double *unused, double *out_4582990555925707440);
void h_27(double *state, double *unused, double *out_7423915965897322824);
void H_27(double *state, double *unused, double *out_7550659603488840876);
void h_29(double *state, double *unused, double *out_1042200623975861930);
void H_29(double *state, double *unused, double *out_74094186150282984);
void h_28(double *state, double *unused, double *out_7646773803679705578);
void H_28(double *state, double *unused, double *out_8696243769011257250);
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
