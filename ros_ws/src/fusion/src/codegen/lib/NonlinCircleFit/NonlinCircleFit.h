//
// File: NonlinCircleFit.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-May-2025 08:58:33
//

#ifndef NONLINCIRCLEFIT_H
#define NONLINCIRCLEFIT_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void NonlinCircleFit(
    double MnvrActive, double TimeInMnvr, double tanRWA, double tanRWA_prev,
    const double desPathX[50], const double desPathY[50],
    const double desPhi[50], double PPursuit_tun1, double L_preview,
    const double veh_param[2], double brent_tol, double brent_itmax,
    double brent_zeps, double wdist, double wangle, const double weight[50],
    double method4ini, double *tandelta_NF, double *costfun_value,
    double *number_iter, double *brent_valid, double *j_preview, double *delta,
    double *tandelta_PP, double *cost, double *cost_deriv, double *Nwp1,
    double *bx, double *cost_dd0_2, double *tandelta_PP2);

#endif
//
// File trailer for NonlinCircleFit.h
//
// [EOF]
//
