/* Produced by CVXGEN, 2020-11-16 10:45:36 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: ldl.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
/* Be sure to place ldl_solve first, so storage schemes are defined by it. */
void ldl_solve(double *target, double *var) {
  int i;
  /* Find var = (L*diag(work.d)*L') \ target, then unpermute. */
  /* Answer goes into var. */
  /* Forward substitution. */
  /* Include permutation as we retrieve from target. Use v so we can unpermute */
  /* later. */
  work.v[0] = target[0];
  work.v[1] = target[1];
  work.v[2] = target[2];
  work.v[3] = target[3]-work.L[0]*work.v[0]-work.L[1]*work.v[1]-work.L[2]*work.v[2];
  /* Diagonal scaling. Assume correctness of work.d_inv. */
  for (i = 0; i < 4; i++)
    work.v[i] *= work.d_inv[i];
  /* Back substitution */
  work.v[2] -= work.L[2]*work.v[3];
  work.v[1] -= work.L[1]*work.v[3];
  work.v[0] -= work.L[0]*work.v[3];
  /* Unpermute the result, from v to var. */
  var[0] = work.v[0];
  var[1] = work.v[1];
  var[2] = work.v[2];
  var[3] = work.v[3];
#ifndef ZERO_LIBRARY_MODE
  if (settings.debug) {
    printf("Squared norm for solution is %.8g.\n", check_residual(target, var));
  }
#endif
}
void ldl_factor(void) {
  work.d[0] = work.KKT[0];
  if (work.d[0] < 0)
    work.d[0] = settings.kkt_reg;
  else
    work.d[0] += settings.kkt_reg;
  work.d_inv[0] = 1/work.d[0];
  work.L[0] = work.KKT[1]*work.d_inv[0];
  work.v[1] = work.KKT[2];
  work.d[1] = work.v[1];
  if (work.d[1] < 0)
    work.d[1] = settings.kkt_reg;
  else
    work.d[1] += settings.kkt_reg;
  work.d_inv[1] = 1/work.d[1];
  work.L[1] = (work.KKT[3])*work.d_inv[1];
  work.v[2] = work.KKT[4];
  work.d[2] = work.v[2];
  if (work.d[2] < 0)
    work.d[2] = settings.kkt_reg;
  else
    work.d[2] += settings.kkt_reg;
  work.d_inv[2] = 1/work.d[2];
  work.L[2] = (work.KKT[5])*work.d_inv[2];
  work.v[0] = work.L[0]*work.d[0];
  work.v[1] = work.L[1]*work.d[1];
  work.v[2] = work.L[2]*work.d[2];
  work.v[3] = work.KKT[6]-work.L[0]*work.v[0]-work.L[1]*work.v[1]-work.L[2]*work.v[2];
  work.d[3] = work.v[3];
  if (work.d[3] > 0)
    work.d[3] = -settings.kkt_reg;
  else
    work.d[3] -= settings.kkt_reg;
  work.d_inv[3] = 1/work.d[3];
#ifndef ZERO_LIBRARY_MODE
  if (settings.debug) {
    printf("Squared Frobenius for factorization is %.8g.\n", check_factorization());
  }
#endif
}
double check_factorization(void) {
  /* Returns the squared Frobenius norm of A - L*D*L'. */
  double temp, residual;
  /* Only check the lower triangle. */
  residual = 0;
  temp = work.KKT[0]-1*work.d[0]*1;
  residual += temp*temp;
  temp = work.KKT[2]-1*work.d[1]*1;
  residual += temp*temp;
  temp = work.KKT[4]-1*work.d[2]*1;
  residual += temp*temp;
  temp = work.KKT[5]-work.L[2]*work.d[2]*1;
  residual += temp*temp;
  temp = work.KKT[6]-work.L[2]*work.d[2]*work.L[2]-1*work.d[3]*1-work.L[0]*work.d[0]*work.L[0]-work.L[1]*work.d[1]*work.L[1];
  residual += temp*temp;
  temp = work.KKT[1]-work.L[0]*work.d[0]*1;
  residual += temp*temp;
  temp = work.KKT[3]-work.L[1]*work.d[1]*1;
  residual += temp*temp;
  return residual;
}
void matrix_multiply(double *result, double *source) {
  /* Finds result = A*source. */
  result[0] = work.KKT[0]*source[0]+work.KKT[1]*source[3];
  result[1] = work.KKT[2]*source[1]+work.KKT[3]*source[3];
  result[2] = work.KKT[4]*source[2]+work.KKT[5]*source[3];
  result[3] = work.KKT[5]*source[2]+work.KKT[6]*source[3]+work.KKT[1]*source[0]+work.KKT[3]*source[1];
}
double check_residual(double *target, double *multiplicand) {
  /* Returns the squared 2-norm of lhs - A*rhs. */
  /* Reuses v to find the residual. */
  int i;
  double residual;
  residual = 0;
  matrix_multiply(work.v, multiplicand);
  for (i = 0; i < 2; i++) {
    residual += (target[i] - work.v[i])*(target[i] - work.v[i]);
  }
  return residual;
}
void fill_KKT(void) {
  work.KKT[0] = 2;
  work.KKT[2] = 2;
  work.KKT[4] = work.s_inv_z[0];
  work.KKT[5] = 1;
  work.KKT[6] = work.block_33[0];
  work.KKT[1] = -params.tau[0]*params.dotx[0];
  work.KKT[3] = -params.tau[0]*params.dotx[1];
}
