
extern "C"{

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

}
extern "C" {
#include <math.h>
/******************************************************************************
 *                       Code generated with sympy 1.4                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3096262396608016595) {
   out_3096262396608016595[0] = delta_x[0] + nom_x[0];
   out_3096262396608016595[1] = delta_x[1] + nom_x[1];
   out_3096262396608016595[2] = delta_x[2] + nom_x[2];
   out_3096262396608016595[3] = delta_x[3] + nom_x[3];
   out_3096262396608016595[4] = delta_x[4] + nom_x[4];
   out_3096262396608016595[5] = delta_x[5] + nom_x[5];
   out_3096262396608016595[6] = delta_x[6] + nom_x[6];
   out_3096262396608016595[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6808884815189872555) {
   out_6808884815189872555[0] = -nom_x[0] + true_x[0];
   out_6808884815189872555[1] = -nom_x[1] + true_x[1];
   out_6808884815189872555[2] = -nom_x[2] + true_x[2];
   out_6808884815189872555[3] = -nom_x[3] + true_x[3];
   out_6808884815189872555[4] = -nom_x[4] + true_x[4];
   out_6808884815189872555[5] = -nom_x[5] + true_x[5];
   out_6808884815189872555[6] = -nom_x[6] + true_x[6];
   out_6808884815189872555[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_8151182089197888992) {
   out_8151182089197888992[0] = 1.0;
   out_8151182089197888992[1] = 0.0;
   out_8151182089197888992[2] = 0.0;
   out_8151182089197888992[3] = 0.0;
   out_8151182089197888992[4] = 0.0;
   out_8151182089197888992[5] = 0.0;
   out_8151182089197888992[6] = 0.0;
   out_8151182089197888992[7] = 0.0;
   out_8151182089197888992[8] = 0.0;
   out_8151182089197888992[9] = 1.0;
   out_8151182089197888992[10] = 0.0;
   out_8151182089197888992[11] = 0.0;
   out_8151182089197888992[12] = 0.0;
   out_8151182089197888992[13] = 0.0;
   out_8151182089197888992[14] = 0.0;
   out_8151182089197888992[15] = 0.0;
   out_8151182089197888992[16] = 0.0;
   out_8151182089197888992[17] = 0.0;
   out_8151182089197888992[18] = 1.0;
   out_8151182089197888992[19] = 0.0;
   out_8151182089197888992[20] = 0.0;
   out_8151182089197888992[21] = 0.0;
   out_8151182089197888992[22] = 0.0;
   out_8151182089197888992[23] = 0.0;
   out_8151182089197888992[24] = 0.0;
   out_8151182089197888992[25] = 0.0;
   out_8151182089197888992[26] = 0.0;
   out_8151182089197888992[27] = 1.0;
   out_8151182089197888992[28] = 0.0;
   out_8151182089197888992[29] = 0.0;
   out_8151182089197888992[30] = 0.0;
   out_8151182089197888992[31] = 0.0;
   out_8151182089197888992[32] = 0.0;
   out_8151182089197888992[33] = 0.0;
   out_8151182089197888992[34] = 0.0;
   out_8151182089197888992[35] = 0.0;
   out_8151182089197888992[36] = 1.0;
   out_8151182089197888992[37] = 0.0;
   out_8151182089197888992[38] = 0.0;
   out_8151182089197888992[39] = 0.0;
   out_8151182089197888992[40] = 0.0;
   out_8151182089197888992[41] = 0.0;
   out_8151182089197888992[42] = 0.0;
   out_8151182089197888992[43] = 0.0;
   out_8151182089197888992[44] = 0.0;
   out_8151182089197888992[45] = 1.0;
   out_8151182089197888992[46] = 0.0;
   out_8151182089197888992[47] = 0.0;
   out_8151182089197888992[48] = 0.0;
   out_8151182089197888992[49] = 0.0;
   out_8151182089197888992[50] = 0.0;
   out_8151182089197888992[51] = 0.0;
   out_8151182089197888992[52] = 0.0;
   out_8151182089197888992[53] = 0.0;
   out_8151182089197888992[54] = 1.0;
   out_8151182089197888992[55] = 0.0;
   out_8151182089197888992[56] = 0.0;
   out_8151182089197888992[57] = 0.0;
   out_8151182089197888992[58] = 0.0;
   out_8151182089197888992[59] = 0.0;
   out_8151182089197888992[60] = 0.0;
   out_8151182089197888992[61] = 0.0;
   out_8151182089197888992[62] = 0.0;
   out_8151182089197888992[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_8080945692066930072) {
   out_8080945692066930072[0] = state[0];
   out_8080945692066930072[1] = state[1];
   out_8080945692066930072[2] = state[2];
   out_8080945692066930072[3] = state[3];
   out_8080945692066930072[4] = state[4];
   out_8080945692066930072[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8080945692066930072[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8080945692066930072[7] = state[7];
}
void F_fun(double *state, double dt, double *out_7407822109033065220) {
   out_7407822109033065220[0] = 1;
   out_7407822109033065220[1] = 0;
   out_7407822109033065220[2] = 0;
   out_7407822109033065220[3] = 0;
   out_7407822109033065220[4] = 0;
   out_7407822109033065220[5] = 0;
   out_7407822109033065220[6] = 0;
   out_7407822109033065220[7] = 0;
   out_7407822109033065220[8] = 0;
   out_7407822109033065220[9] = 1;
   out_7407822109033065220[10] = 0;
   out_7407822109033065220[11] = 0;
   out_7407822109033065220[12] = 0;
   out_7407822109033065220[13] = 0;
   out_7407822109033065220[14] = 0;
   out_7407822109033065220[15] = 0;
   out_7407822109033065220[16] = 0;
   out_7407822109033065220[17] = 0;
   out_7407822109033065220[18] = 1;
   out_7407822109033065220[19] = 0;
   out_7407822109033065220[20] = 0;
   out_7407822109033065220[21] = 0;
   out_7407822109033065220[22] = 0;
   out_7407822109033065220[23] = 0;
   out_7407822109033065220[24] = 0;
   out_7407822109033065220[25] = 0;
   out_7407822109033065220[26] = 0;
   out_7407822109033065220[27] = 1;
   out_7407822109033065220[28] = 0;
   out_7407822109033065220[29] = 0;
   out_7407822109033065220[30] = 0;
   out_7407822109033065220[31] = 0;
   out_7407822109033065220[32] = 0;
   out_7407822109033065220[33] = 0;
   out_7407822109033065220[34] = 0;
   out_7407822109033065220[35] = 0;
   out_7407822109033065220[36] = 1;
   out_7407822109033065220[37] = 0;
   out_7407822109033065220[38] = 0;
   out_7407822109033065220[39] = 0;
   out_7407822109033065220[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7407822109033065220[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7407822109033065220[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7407822109033065220[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7407822109033065220[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7407822109033065220[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7407822109033065220[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7407822109033065220[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7407822109033065220[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7407822109033065220[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7407822109033065220[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7407822109033065220[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7407822109033065220[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7407822109033065220[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7407822109033065220[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7407822109033065220[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7407822109033065220[56] = 0;
   out_7407822109033065220[57] = 0;
   out_7407822109033065220[58] = 0;
   out_7407822109033065220[59] = 0;
   out_7407822109033065220[60] = 0;
   out_7407822109033065220[61] = 0;
   out_7407822109033065220[62] = 0;
   out_7407822109033065220[63] = 1;
}
void h_25(double *state, double *unused, double *out_2684688280309935765) {
   out_2684688280309935765[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8884442395634928840) {
   out_8884442395634928840[0] = 0;
   out_8884442395634928840[1] = 0;
   out_8884442395634928840[2] = 0;
   out_8884442395634928840[3] = 0;
   out_8884442395634928840[4] = 0;
   out_8884442395634928840[5] = 0;
   out_8884442395634928840[6] = 1;
   out_8884442395634928840[7] = 0;
}
void h_24(double *state, double *unused, double *out_230005947588702794) {
   out_230005947588702794[0] = state[4];
   out_230005947588702794[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7275217207707909980) {
   out_7275217207707909980[0] = 0;
   out_7275217207707909980[1] = 0;
   out_7275217207707909980[2] = 0;
   out_7275217207707909980[3] = 0;
   out_7275217207707909980[4] = 1;
   out_7275217207707909980[5] = 0;
   out_7275217207707909980[6] = 0;
   out_7275217207707909980[7] = 0;
   out_7275217207707909980[8] = 0;
   out_7275217207707909980[9] = 0;
   out_7275217207707909980[10] = 0;
   out_7275217207707909980[11] = 0;
   out_7275217207707909980[12] = 0;
   out_7275217207707909980[13] = 1;
   out_7275217207707909980[14] = 0;
   out_7275217207707909980[15] = 0;
}
void h_30(double *state, double *unused, double *out_4935230695321287415) {
   out_4935230695321287415[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7262152278745433074) {
   out_7262152278745433074[0] = 0;
   out_7262152278745433074[1] = 0;
   out_7262152278745433074[2] = 0;
   out_7262152278745433074[3] = 0;
   out_7262152278745433074[4] = 1;
   out_7262152278745433074[5] = 0;
   out_7262152278745433074[6] = 0;
   out_7262152278745433074[7] = 0;
}
void h_26(double *state, double *unused, double *out_2615980980938838772) {
   out_2615980980938838772[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4582990555925707440) {
   out_4582990555925707440[0] = 0;
   out_4582990555925707440[1] = 0;
   out_4582990555925707440[2] = 0;
   out_4582990555925707440[3] = 0;
   out_4582990555925707440[4] = 0;
   out_4582990555925707440[5] = 0;
   out_4582990555925707440[6] = 0;
   out_4582990555925707440[7] = 1;
}
void h_27(double *state, double *unused, double *out_7423915965897322824) {
   out_7423915965897322824[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7550659603488840876) {
   out_7550659603488840876[0] = 0;
   out_7550659603488840876[1] = 0;
   out_7550659603488840876[2] = 0;
   out_7550659603488840876[3] = 1;
   out_7550659603488840876[4] = 0;
   out_7550659603488840876[5] = 0;
   out_7550659603488840876[6] = 0;
   out_7550659603488840876[7] = 0;
}
void h_29(double *state, double *unused, double *out_1042200623975861930) {
   out_1042200623975861930[0] = state[1];
}
void H_29(double *state, double *unused, double *out_74094186150282984) {
   out_74094186150282984[0] = 0;
   out_74094186150282984[1] = 1;
   out_74094186150282984[2] = 0;
   out_74094186150282984[3] = 0;
   out_74094186150282984[4] = 0;
   out_74094186150282984[5] = 0;
   out_74094186150282984[6] = 0;
   out_74094186150282984[7] = 0;
}
void h_28(double *state, double *unused, double *out_7646773803679705578) {
   out_7646773803679705578[0] = state[5];
   out_7646773803679705578[1] = state[6];
}
void H_28(double *state, double *unused, double *out_8696243769011257250) {
   out_8696243769011257250[0] = 0;
   out_8696243769011257250[1] = 0;
   out_8696243769011257250[2] = 0;
   out_8696243769011257250[3] = 0;
   out_8696243769011257250[4] = 0;
   out_8696243769011257250[5] = 1;
   out_8696243769011257250[6] = 0;
   out_8696243769011257250[7] = 0;
   out_8696243769011257250[8] = 0;
   out_8696243769011257250[9] = 0;
   out_8696243769011257250[10] = 0;
   out_8696243769011257250[11] = 0;
   out_8696243769011257250[12] = 0;
   out_8696243769011257250[13] = 0;
   out_8696243769011257250[14] = 1;
   out_8696243769011257250[15] = 0;
}
}

extern "C"{
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



extern "C"{

      void update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
      }
    
      void update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<2,3,0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
      }
    
      void update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
      }
    
      void update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
      }
    
      void update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
      }
    
      void update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
      }
    
      void update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<2,3,0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
      }
    
}
