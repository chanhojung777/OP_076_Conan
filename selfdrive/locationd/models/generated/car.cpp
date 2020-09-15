
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
void err_fun(double *nom_x, double *delta_x, double *out_727470535757866170) {
   out_727470535757866170[0] = delta_x[0] + nom_x[0];
   out_727470535757866170[1] = delta_x[1] + nom_x[1];
   out_727470535757866170[2] = delta_x[2] + nom_x[2];
   out_727470535757866170[3] = delta_x[3] + nom_x[3];
   out_727470535757866170[4] = delta_x[4] + nom_x[4];
   out_727470535757866170[5] = delta_x[5] + nom_x[5];
   out_727470535757866170[6] = delta_x[6] + nom_x[6];
   out_727470535757866170[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7823608325048859542) {
   out_7823608325048859542[0] = -nom_x[0] + true_x[0];
   out_7823608325048859542[1] = -nom_x[1] + true_x[1];
   out_7823608325048859542[2] = -nom_x[2] + true_x[2];
   out_7823608325048859542[3] = -nom_x[3] + true_x[3];
   out_7823608325048859542[4] = -nom_x[4] + true_x[4];
   out_7823608325048859542[5] = -nom_x[5] + true_x[5];
   out_7823608325048859542[6] = -nom_x[6] + true_x[6];
   out_7823608325048859542[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_8561215076071933433) {
   out_8561215076071933433[0] = 1.0;
   out_8561215076071933433[1] = 0.0;
   out_8561215076071933433[2] = 0.0;
   out_8561215076071933433[3] = 0.0;
   out_8561215076071933433[4] = 0.0;
   out_8561215076071933433[5] = 0.0;
   out_8561215076071933433[6] = 0.0;
   out_8561215076071933433[7] = 0.0;
   out_8561215076071933433[8] = 0.0;
   out_8561215076071933433[9] = 1.0;
   out_8561215076071933433[10] = 0.0;
   out_8561215076071933433[11] = 0.0;
   out_8561215076071933433[12] = 0.0;
   out_8561215076071933433[13] = 0.0;
   out_8561215076071933433[14] = 0.0;
   out_8561215076071933433[15] = 0.0;
   out_8561215076071933433[16] = 0.0;
   out_8561215076071933433[17] = 0.0;
   out_8561215076071933433[18] = 1.0;
   out_8561215076071933433[19] = 0.0;
   out_8561215076071933433[20] = 0.0;
   out_8561215076071933433[21] = 0.0;
   out_8561215076071933433[22] = 0.0;
   out_8561215076071933433[23] = 0.0;
   out_8561215076071933433[24] = 0.0;
   out_8561215076071933433[25] = 0.0;
   out_8561215076071933433[26] = 0.0;
   out_8561215076071933433[27] = 1.0;
   out_8561215076071933433[28] = 0.0;
   out_8561215076071933433[29] = 0.0;
   out_8561215076071933433[30] = 0.0;
   out_8561215076071933433[31] = 0.0;
   out_8561215076071933433[32] = 0.0;
   out_8561215076071933433[33] = 0.0;
   out_8561215076071933433[34] = 0.0;
   out_8561215076071933433[35] = 0.0;
   out_8561215076071933433[36] = 1.0;
   out_8561215076071933433[37] = 0.0;
   out_8561215076071933433[38] = 0.0;
   out_8561215076071933433[39] = 0.0;
   out_8561215076071933433[40] = 0.0;
   out_8561215076071933433[41] = 0.0;
   out_8561215076071933433[42] = 0.0;
   out_8561215076071933433[43] = 0.0;
   out_8561215076071933433[44] = 0.0;
   out_8561215076071933433[45] = 1.0;
   out_8561215076071933433[46] = 0.0;
   out_8561215076071933433[47] = 0.0;
   out_8561215076071933433[48] = 0.0;
   out_8561215076071933433[49] = 0.0;
   out_8561215076071933433[50] = 0.0;
   out_8561215076071933433[51] = 0.0;
   out_8561215076071933433[52] = 0.0;
   out_8561215076071933433[53] = 0.0;
   out_8561215076071933433[54] = 1.0;
   out_8561215076071933433[55] = 0.0;
   out_8561215076071933433[56] = 0.0;
   out_8561215076071933433[57] = 0.0;
   out_8561215076071933433[58] = 0.0;
   out_8561215076071933433[59] = 0.0;
   out_8561215076071933433[60] = 0.0;
   out_8561215076071933433[61] = 0.0;
   out_8561215076071933433[62] = 0.0;
   out_8561215076071933433[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_5854494543467114630) {
   out_5854494543467114630[0] = state[0];
   out_5854494543467114630[1] = state[1];
   out_5854494543467114630[2] = state[2];
   out_5854494543467114630[3] = state[3];
   out_5854494543467114630[4] = state[4];
   out_5854494543467114630[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5854494543467114630[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5854494543467114630[7] = state[7];
}
void F_fun(double *state, double dt, double *out_7214143257728792534) {
   out_7214143257728792534[0] = 1;
   out_7214143257728792534[1] = 0;
   out_7214143257728792534[2] = 0;
   out_7214143257728792534[3] = 0;
   out_7214143257728792534[4] = 0;
   out_7214143257728792534[5] = 0;
   out_7214143257728792534[6] = 0;
   out_7214143257728792534[7] = 0;
   out_7214143257728792534[8] = 0;
   out_7214143257728792534[9] = 1;
   out_7214143257728792534[10] = 0;
   out_7214143257728792534[11] = 0;
   out_7214143257728792534[12] = 0;
   out_7214143257728792534[13] = 0;
   out_7214143257728792534[14] = 0;
   out_7214143257728792534[15] = 0;
   out_7214143257728792534[16] = 0;
   out_7214143257728792534[17] = 0;
   out_7214143257728792534[18] = 1;
   out_7214143257728792534[19] = 0;
   out_7214143257728792534[20] = 0;
   out_7214143257728792534[21] = 0;
   out_7214143257728792534[22] = 0;
   out_7214143257728792534[23] = 0;
   out_7214143257728792534[24] = 0;
   out_7214143257728792534[25] = 0;
   out_7214143257728792534[26] = 0;
   out_7214143257728792534[27] = 1;
   out_7214143257728792534[28] = 0;
   out_7214143257728792534[29] = 0;
   out_7214143257728792534[30] = 0;
   out_7214143257728792534[31] = 0;
   out_7214143257728792534[32] = 0;
   out_7214143257728792534[33] = 0;
   out_7214143257728792534[34] = 0;
   out_7214143257728792534[35] = 0;
   out_7214143257728792534[36] = 1;
   out_7214143257728792534[37] = 0;
   out_7214143257728792534[38] = 0;
   out_7214143257728792534[39] = 0;
   out_7214143257728792534[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7214143257728792534[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7214143257728792534[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7214143257728792534[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7214143257728792534[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7214143257728792534[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7214143257728792534[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7214143257728792534[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7214143257728792534[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7214143257728792534[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7214143257728792534[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7214143257728792534[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7214143257728792534[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7214143257728792534[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7214143257728792534[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7214143257728792534[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7214143257728792534[56] = 0;
   out_7214143257728792534[57] = 0;
   out_7214143257728792534[58] = 0;
   out_7214143257728792534[59] = 0;
   out_7214143257728792534[60] = 0;
   out_7214143257728792534[61] = 0;
   out_7214143257728792534[62] = 0;
   out_7214143257728792534[63] = 1;
}
void h_25(double *state, double *unused, double *out_604571298292658417) {
   out_604571298292658417[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3286662799098636255) {
   out_3286662799098636255[0] = 0;
   out_3286662799098636255[1] = 0;
   out_3286662799098636255[2] = 0;
   out_3286662799098636255[3] = 0;
   out_3286662799098636255[4] = 0;
   out_3286662799098636255[5] = 0;
   out_3286662799098636255[6] = 1;
   out_3286662799098636255[7] = 0;
}
void h_24(double *state, double *unused, double *out_5542032034878286667) {
   out_5542032034878286667[0] = state[4];
   out_5542032034878286667[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3810660555687201395) {
   out_3810660555687201395[0] = 0;
   out_3810660555687201395[1] = 0;
   out_3810660555687201395[2] = 0;
   out_3810660555687201395[3] = 0;
   out_3810660555687201395[4] = 1;
   out_3810660555687201395[5] = 0;
   out_3810660555687201395[6] = 0;
   out_3810660555687201395[7] = 0;
   out_3810660555687201395[8] = 0;
   out_3810660555687201395[9] = 0;
   out_3810660555687201395[10] = 0;
   out_3810660555687201395[11] = 0;
   out_3810660555687201395[12] = 0;
   out_3810660555687201395[13] = 1;
   out_3810660555687201395[14] = 0;
   out_3810660555687201395[15] = 0;
}
void h_30(double *state, double *unused, double *out_4521560671013167731) {
   out_4521560671013167731[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2965787034756330037) {
   out_2965787034756330037[0] = 0;
   out_2965787034756330037[1] = 0;
   out_2965787034756330037[2] = 0;
   out_2965787034756330037[3] = 0;
   out_2965787034756330037[4] = 1;
   out_2965787034756330037[5] = 0;
   out_2965787034756330037[6] = 0;
   out_2965787034756330037[7] = 0;
}
void h_26(double *state, double *unused, double *out_8202605294476525334) {
   out_8202605294476525334[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2359552449907263145) {
   out_2359552449907263145[0] = 0;
   out_2359552449907263145[1] = 0;
   out_2359552449907263145[2] = 0;
   out_2359552449907263145[3] = 0;
   out_2359552449907263145[4] = 0;
   out_2359552449907263145[5] = 0;
   out_2359552449907263145[6] = 0;
   out_2359552449907263145[7] = 1;
}
void h_27(double *state, double *unused, double *out_7009077137081734482) {
   out_7009077137081734482[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4843577531083537915) {
   out_4843577531083537915[0] = 0;
   out_4843577531083537915[1] = 0;
   out_4843577531083537915[2] = 0;
   out_4843577531083537915[3] = 1;
   out_4843577531083537915[4] = 0;
   out_4843577531083537915[5] = 0;
   out_4843577531083537915[6] = 0;
   out_4843577531083537915[7] = 0;
}
void h_29(double *state, double *unused, double *out_3640713933842738224) {
   out_3640713933842738224[0] = state[1];
}
void H_29(double *state, double *unused, double *out_389076561956202671) {
   out_389076561956202671[0] = 0;
   out_389076561956202671[1] = 1;
   out_389076561956202671[2] = 0;
   out_389076561956202671[3] = 0;
   out_389076561956202671[4] = 0;
   out_389076561956202671[5] = 0;
   out_389076561956202671[6] = 0;
   out_389076561956202671[7] = 0;
}
void h_28(double *state, double *unused, double *out_5364533368482937023) {
   out_5364533368482937023[0] = state[5];
   out_5364533368482937023[1] = state[6];
}
void H_28(double *state, double *unused, double *out_7941601476428430007) {
   out_7941601476428430007[0] = 0;
   out_7941601476428430007[1] = 0;
   out_7941601476428430007[2] = 0;
   out_7941601476428430007[3] = 0;
   out_7941601476428430007[4] = 0;
   out_7941601476428430007[5] = 1;
   out_7941601476428430007[6] = 0;
   out_7941601476428430007[7] = 0;
   out_7941601476428430007[8] = 0;
   out_7941601476428430007[9] = 0;
   out_7941601476428430007[10] = 0;
   out_7941601476428430007[11] = 0;
   out_7941601476428430007[12] = 0;
   out_7941601476428430007[13] = 0;
   out_7941601476428430007[14] = 1;
   out_7941601476428430007[15] = 0;
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
