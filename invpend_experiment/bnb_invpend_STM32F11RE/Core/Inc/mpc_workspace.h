#ifndef MPC_WORKSPACE_H
#define MPC_WORKSPACE_H

#include "types.h"
#include "constants.h"
// Settings prototype
extern DAQPSettings settings;

#define DAQP_BNB
extern int bin_ids[12];
extern DAQPNode tree[13];
extern int tree_WS[325];
extern int fixed_ids[13];
extern DAQPBnB daqp_bnb_work;

#undef NX
#define NX 24
#undef N_CONSTR
#define N_CONSTR 96
#undef N_SIMPLE
#define N_SIMPLE 24 
// Workspace prototypes
extern c_float M[1728];
extern c_float dupper[96];
extern c_float dlower[96];
extern c_float Rinv[300];
extern c_float v[24];
extern int sense[96];

extern c_float scaling[96];

extern c_float x[25];
extern c_float xold[25];

extern c_float lam[25];
extern c_float lam_star[25];
extern c_float u[25];

extern c_float L[325];
extern c_float D[25];
extern c_float xldl[25];
extern c_float zldl[25];

extern int WS[25];

extern DAQPWorkspace daqp_work;

#endif // ifndef MPC_WORKSPACE_H
#ifndef MPC_WORKSPACE_MPC_H
#define MPC_WORKSPACE_MPC_H

#define N_THETA 4
#define N_CONTROL 4

extern c_float Dth[384];
extern c_float du[96];
extern c_float dl[96];

extern c_float Xth[16];

void mpc_update_qp(c_float* th, c_float* dupper, c_float* dlower);
void mpc_get_solution(c_float* th, c_float* control, c_float* xstar);
int mpc_compute_control(c_float* th, c_float* control, DAQPWorkspace* work);
#endif // ifndef MPC_WORKSPACE_MPC_H
