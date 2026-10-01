//
// File: spline.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 22-May-2025 13:40:44
//

// Include Files
#include "spline.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const double x[100]
//                const double y[100]
//                const double xx[50]
//                double output[50]
// Return Type  : void
//
namespace coder {
void spline(const double x[100], const double y[100], const double xx[50],
            double output[50])
{
  double pp_coefs[396];
  double md[100];
  double s[100];
  double dvdf[99];
  double dx[99];
  double d;
  double d31;
  double dnnm2;
  double r;
  for (int k{0}; k < 99; k++) {
    d = x[k + 1] - x[k];
    dx[k] = d;
    dvdf[k] = (y[k + 1] - y[k]) / d;
  }
  d31 = x[2] - x[0];
  dnnm2 = x[99] - x[97];
  s[0] =
      ((dx[0] + 2.0 * d31) * dx[1] * dvdf[0] + dx[0] * dx[0] * dvdf[1]) / d31;
  s[99] = ((dx[98] + 2.0 * dnnm2) * dx[97] * dvdf[98] +
           dx[98] * dx[98] * dvdf[97]) /
          dnnm2;
  md[0] = dx[1];
  md[99] = dx[97];
  for (int k{0}; k < 98; k++) {
    r = dx[k + 1];
    d = dx[k];
    s[k + 1] = 3.0 * (r * dvdf[k] + d * dvdf[k + 1]);
    md[k + 1] = 2.0 * (r + d);
  }
  r = dx[1] / md[0];
  md[1] -= r * d31;
  s[1] -= r * s[0];
  for (int k{0}; k < 97; k++) {
    r = dx[k + 2] / md[k + 1];
    md[k + 2] -= r * dx[k];
    s[k + 2] -= r * s[k + 1];
  }
  r = dnnm2 / md[98];
  md[99] -= r * dx[97];
  s[99] -= r * s[98];
  s[99] /= md[99];
  for (int k{97}; k >= 0; k--) {
    s[k + 1] = (s[k + 1] - dx[k] * s[k + 2]) / md[k + 1];
  }
  s[0] = (s[0] - d31 * s[1]) / md[0];
  for (int k{0}; k < 99; k++) {
    double dzzdx;
    d = dvdf[k];
    d31 = s[k];
    dnnm2 = dx[k];
    dzzdx = (d - d31) / dnnm2;
    r = (s[k + 1] - d) / dnnm2;
    pp_coefs[k] = (r - dzzdx) / dnnm2;
    pp_coefs[k + 99] = 2.0 * dzzdx - r;
    pp_coefs[k + 198] = d31;
    pp_coefs[k + 297] = y[k];
  }
  for (int k{0}; k < 50; k++) {
    r = xx[k];
    if (!std::isnan(r)) {
      int high_i;
      int low_i;
      int low_ip1;
      low_i = 0;
      low_ip1 = 2;
      high_i = 100;
      while (high_i > low_ip1) {
        int mid_i;
        mid_i = ((low_i + high_i) + 1) >> 1;
        if (xx[k] >= x[mid_i - 1]) {
          low_i = mid_i - 1;
          low_ip1 = mid_i + 1;
        } else {
          high_i = mid_i;
        }
      }
      r = xx[k] - x[low_i];
      r = r * (r * (r * pp_coefs[low_i] + pp_coefs[low_i + 99]) +
               pp_coefs[low_i + 198]) +
          pp_coefs[low_i + 297];
    }
    output[k] = r;
  }
}

} // namespace coder

//
// File trailer for spline.cpp
//
// [EOF]
//
