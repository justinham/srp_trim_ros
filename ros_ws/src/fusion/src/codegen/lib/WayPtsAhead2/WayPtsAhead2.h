//
// File: WayPtsAhead2.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 22-May-2025 13:40:44
//

#ifndef WAYPTSAHEAD2_H
#define WAYPTSAHEAD2_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void WayPtsAhead2(double localx, double localy, double localh, double delta,
                  double* path_x, double* path_y,
                  double* path_h, const int traj_len, double *dis,
                  unsigned short *b_index, double *imode, double xwp[50],
                  double ywp[50], double hwp[50], double xwp2[50],
                  double ywp2[50], double hwp2[50]);

#endif
//
// File trailer for WayPtsAhead2.h
//
// [EOF]
//