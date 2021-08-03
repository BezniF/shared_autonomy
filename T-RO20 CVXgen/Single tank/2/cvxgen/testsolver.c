/* Produced by CVXGEN, 2020-12-15 06:29:26 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.Fd[0] = 0.20319161029830202;
  params.Fd[1] = 0.8325912904724193;
  params.Fd[2] = -0.8363810443482227;
  params.Fd[3] = 0.04331042079065206;
  params.Fd[4] = 1.5717878173906188;
  params.Fd[5] = 1.5851723557337523;
  params.Fd[6] = -1.497658758144655;
  params.Fd[7] = -1.171028487447253;
  params.Fd[8] = -1.7941311867966805;
  params.Fd[9] = -0.23676062539745413;
  params.Fd[10] = -1.8804951564857322;
  params.Fd[11] = -0.17266710242115568;
  params.tau[0] = 0.596576190459043;
  params.dotx[0] = -0.8860508694080989;
  params.dotx[1] = 0.7050196079205251;
  params.dotx[2] = 0.3634512696654033;
  params.dotx[3] = -1.9040724704913385;
  params.dotx[4] = 0.23541635196352795;
  params.dotx[5] = -0.9629902123701384;
  params.dotx[6] = -0.3395952119597214;
  params.dotx[7] = -0.865899672914725;
  params.dotx[8] = 0.7725516732519853;
  params.dotx[9] = -0.23818512931704205;
  params.dotx[10] = -1.372529046100147;
  params.dotx[11] = 0.17859607212737894;
  params.T0[0] = 1.1212590580454682;
  params.varepsilon[0] = -0.774545870495281;
}
