/* Produced by CVXGEN, 2021-10-01 12:33:22 -0400.  */
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
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.tau[0]*params.dotx[0])-rhs[1]*(params.tau[0]*params.dotx[1])-rhs[2]*(params.tau[0]*params.dotx[2])-rhs[3]*(params.tau[0]*params.dotx[3]);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.tau[0]*params.dotx[0]);
  lhs[1] = -rhs[0]*(params.tau[0]*params.dotx[1]);
  lhs[2] = -rhs[0]*(params.tau[0]*params.dotx[2]);
  lhs[3] = -rhs[0]*(params.tau[0]*params.dotx[3]);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2);
  lhs[1] = rhs[1]*(2);
  lhs[2] = rhs[2]*(2);
  lhs[3] = rhs[3]*(2);
}
void fillq(void) {
  work.q[0] = -2*params.Fd[0];
  work.q[1] = -2*params.Fd[1];
  work.q[2] = -2*params.Fd[2];
  work.q[3] = -2*params.Fd[3];
}
void fillh(void) {
  work.h[0] = -(-params.T0[0]+params.varepsilon[0]+params.tau[0]*(params.P_in[0]-params.P_out[0]));
}
void fillb(void) {
}
void pre_ops(void) {
  work.quad_759852212224[0] = params.Fd[0]*params.Fd[0]+params.Fd[1]*params.Fd[1]+params.Fd[2]*params.Fd[2]+params.Fd[3]*params.Fd[3];
}
