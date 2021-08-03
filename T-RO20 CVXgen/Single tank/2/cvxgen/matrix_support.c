/* Produced by CVXGEN, 2020-12-15 06:29:26 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
  lhs[6] = 0;
  lhs[7] = 0;
  lhs[8] = 0;
  lhs[9] = 0;
  lhs[10] = 0;
  lhs[11] = 0;
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-params.tau[0]*params.dotx[0])-rhs[1]*(-params.tau[0]*params.dotx[1])-rhs[2]*(-params.tau[0]*params.dotx[2])-rhs[3]*(-params.tau[0]*params.dotx[3])-rhs[4]*(-params.tau[0]*params.dotx[4])-rhs[5]*(-params.tau[0]*params.dotx[5])-rhs[6]*(-params.tau[0]*params.dotx[6])-rhs[7]*(-params.tau[0]*params.dotx[7])-rhs[8]*(-params.tau[0]*params.dotx[8])-rhs[9]*(-params.tau[0]*params.dotx[9])-rhs[10]*(-params.tau[0]*params.dotx[10])-rhs[11]*(-params.tau[0]*params.dotx[11]);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-params.tau[0]*params.dotx[0]);
  lhs[1] = -rhs[0]*(-params.tau[0]*params.dotx[1]);
  lhs[2] = -rhs[0]*(-params.tau[0]*params.dotx[2]);
  lhs[3] = -rhs[0]*(-params.tau[0]*params.dotx[3]);
  lhs[4] = -rhs[0]*(-params.tau[0]*params.dotx[4]);
  lhs[5] = -rhs[0]*(-params.tau[0]*params.dotx[5]);
  lhs[6] = -rhs[0]*(-params.tau[0]*params.dotx[6]);
  lhs[7] = -rhs[0]*(-params.tau[0]*params.dotx[7]);
  lhs[8] = -rhs[0]*(-params.tau[0]*params.dotx[8]);
  lhs[9] = -rhs[0]*(-params.tau[0]*params.dotx[9]);
  lhs[10] = -rhs[0]*(-params.tau[0]*params.dotx[10]);
  lhs[11] = -rhs[0]*(-params.tau[0]*params.dotx[11]);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2);
  lhs[1] = rhs[1]*(2);
  lhs[2] = rhs[2]*(2);
  lhs[3] = rhs[3]*(2);
  lhs[4] = rhs[4]*(2);
  lhs[5] = rhs[5]*(2);
  lhs[6] = rhs[6]*(2);
  lhs[7] = rhs[7]*(2);
  lhs[8] = rhs[8]*(2);
  lhs[9] = rhs[9]*(2);
  lhs[10] = rhs[10]*(2);
  lhs[11] = rhs[11]*(2);
}
void fillq(void) {
  work.q[0] = -2*params.Fd[0];
  work.q[1] = -2*params.Fd[1];
  work.q[2] = -2*params.Fd[2];
  work.q[3] = -2*params.Fd[3];
  work.q[4] = -2*params.Fd[4];
  work.q[5] = -2*params.Fd[5];
  work.q[6] = -2*params.Fd[6];
  work.q[7] = -2*params.Fd[7];
  work.q[8] = -2*params.Fd[8];
  work.q[9] = -2*params.Fd[9];
  work.q[10] = -2*params.Fd[10];
  work.q[11] = -2*params.Fd[11];
}
void fillh(void) {
  work.h[0] = -(-params.T0[0]+params.varepsilon[0]);
}
void fillb(void) {
}
void pre_ops(void) {
  work.quad_759852212224[0] = params.Fd[0]*params.Fd[0]+params.Fd[1]*params.Fd[1]+params.Fd[2]*params.Fd[2]+params.Fd[3]*params.Fd[3]+params.Fd[4]*params.Fd[4]+params.Fd[5]*params.Fd[5]+params.Fd[6]*params.Fd[6]+params.Fd[7]*params.Fd[7]+params.Fd[8]*params.Fd[8]+params.Fd[9]*params.Fd[9]+params.Fd[10]*params.Fd[10]+params.Fd[11]*params.Fd[11];
}
