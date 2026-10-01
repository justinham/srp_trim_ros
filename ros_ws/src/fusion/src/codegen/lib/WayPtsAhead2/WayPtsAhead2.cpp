//
// File: WayPtsAhead2.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 22-May-2025 13:40:44
//

// Include Files
#include "WayPtsAhead2.h"
#include "spline.h"
#include <cmath>
#include <cstring>
#include <vector>

// Function Definitions
//
// Arguments    : double localx
//                double localy
//                double localh
//                double delta
//                const double path_x[298]
//                const double path_y[298]
//                const double path_h[298]
//                double *dis
//                unsigned short *b_index
//                double *imode
//                double xwp[50]
//                double ywp[50]
//                double hwp[50]
//                double xwp2[50]
//                double ywp2[50]
//                double hwp2[50]
// Return Type  : void
//
void WayPtsAhead2(double localx, double localy, double localh, double delta,
                  double* path_x, double* path_y,
                  double* path_h, const int traj_len, double *dis,
                  unsigned short *b_index, double *imode, double xwp[50],
                  double ywp[50], double hwp[50], double xwp2[50],
                  double ywp2[50], double hwp2[50])
{
  // std::vector<double> dissq;
  double dissq[traj_len];
  double clength[100];
  double hwp1[100];
  double xwp1[100];
  double ywp1[100];
  double clengthdes[50];
  double b_xwp_tmp_tmp;
  double xwp_tmp;
  double xwp_tmp_tmp;
  double ywp_1;
  int i;
  int idx;
  int k;
  unsigned short ind;
  std::memset(&xwp[0], 0, 50U * sizeof(double));
  std::memset(&ywp[0], 0, 50U * sizeof(double));
  std::memset(&clength[0], 0, 100U * sizeof(double));
  *imode = 0.0;
  //  Demo Case provided by Johnson
  // number of route way points (city course, for example)
  for (k = 0; k < traj_len; k++) {
    xwp_tmp = localx - path_x[k];
    ywp_1 = xwp_tmp * xwp_tmp;
    xwp_tmp = localy - path_y[k];
    ywp_1 += xwp_tmp * xwp_tmp;
    dissq[k] = ywp_1;
    // dissq.push_back(ywp_1);
  }
  if (!std::isnan(dissq[0])) {
    idx = 1;
  } else {
    bool exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < traj_len+1)) {
      if (!std::isnan(dissq[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    ywp_1 = dissq[0];
    idx = 1;
  } else {
    ywp_1 = dissq[idx - 1];
    i = idx + 1;
    for (k = i; k < traj_len+1; k++) {
      xwp_tmp = dissq[k - 1];
      if (ywp_1 > xwp_tmp) {
        ywp_1 = xwp_tmp;
        idx = k;
      }
    }
  }
  *dis = std::sqrt(ywp_1);
  *b_index = static_cast<unsigned short>(idx);
  xwp_tmp_tmp = std::sin(localh);
  b_xwp_tmp_tmp = std::cos(localh);
  for (k = 0; k < 49; k++) {
    ind = static_cast<unsigned short>((idx + k) + 1);
    if (ind != traj_len) {
      ind = static_cast<unsigned short>(static_cast<unsigned int>(ind) -
                                        ind / traj_len * traj_len);
    }
    ywp_1 = path_x[ind - 1] - localx;
    xwp_tmp = path_y[ind - 1] - localy;
    xwp[k + 1] = ywp_1 * b_xwp_tmp_tmp + xwp_tmp * xwp_tmp_tmp;
    ywp[k + 1] = -ywp_1 * xwp_tmp_tmp + xwp_tmp * b_xwp_tmp_tmp;
    hwp[k + 1] = path_h[ind - 1] - localh;
  }
  double xwp_1;
  // for j2=1:50
  hwp[0] = path_h[0] - localh;
  xwp_tmp = path_x[idx - 1] - localx;
  ywp_1 = path_y[idx - 1] - localy;
  xwp_1 = xwp_tmp * b_xwp_tmp_tmp + ywp_1 * xwp_tmp_tmp;
  ywp_1 = -xwp_tmp * xwp_tmp_tmp + ywp_1 * b_xwp_tmp_tmp;
  xwp[0] = 0.0;
  ywp[0] = ywp_1 + (ywp[1] - ywp_1) * (0.0 - xwp_1) / (xwp[1] - xwp_1);
  for (k = 0; k < 100; k++) {
    ind = static_cast<unsigned short>(idx + k);
    if (ind != traj_len) {
      ind = static_cast<unsigned short>(static_cast<unsigned int>(ind) -
                                        ind / traj_len * traj_len);
    }
    xwp_tmp = path_x[ind - 1] - localx;
    ywp_1 = path_y[ind - 1] - localy;
    xwp1[k] = xwp_tmp * b_xwp_tmp_tmp + ywp_1 * xwp_tmp_tmp;
    ywp1[k] = -xwp_tmp * xwp_tmp_tmp + ywp_1 * b_xwp_tmp_tmp;
    hwp1[k] = path_h[ind - 1] - localh;
  }
  // for j2=1:50
  for (k = 0; k < 99; k++) {
    ywp_1 = xwp1[k + 1] - xwp1[k];
    xwp_tmp = ywp1[k + 1] - ywp1[k];
    clength[k + 1] = clength[k] + std::sqrt(ywp_1 * ywp_1 + xwp_tmp * xwp_tmp);
  }
  for (i = 0; i < 50; i++) {
    clengthdes[i] = static_cast<double>(i) * delta;
  }
  coder::spline(clength, xwp1, clengthdes, xwp2);
  coder::spline(clength, ywp1, clengthdes, ywp2);
  coder::spline(clength, hwp1, clengthdes, hwp2);
}

//
// File trailer for WayPtsAhead2.cpp
//
// [EOF]
//