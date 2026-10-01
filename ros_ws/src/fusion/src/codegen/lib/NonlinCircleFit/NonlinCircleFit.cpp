//
// File: NonlinCircleFit.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 13-May-2025 10:58:37
//

// Include Files
#include "NonlinCircleFit.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Declarations
static double Ja(double z, const double x_wp[50], const double y_wp[50],
                 const double h_wp[50], const double w[50], double a, double b,
                 double Nwp);

static double Jadot(double z, const double x_wp[50], const double y_wp[50],
                    const double h_wp[50], const double w[50], double a,
                    double b, double Nwp);

static double Jydot(double z, const double x_wp[50], const double y_wp[50],
                    const double w[50], double a, double b, double Nwp);

static double rt_powd_snf(double u0, double u1);

// Function Definitions
//
// cost function for angular error
//  way point x_wp,y_wp, head_wp of dimension N
//  a+b=L vehicle wheel base
//
// Arguments    : double z
//                const double x_wp[50]
//                const double y_wp[50]
//                const double h_wp[50]
//                const double w[50]
//                double a
//                double b
//                double Nwp
// Return Type  : double
//
static double Ja(double z, const double x_wp[50], const double y_wp[50],
                 const double h_wp[50], const double w[50], double a, double b,
                 double Nwp)
{
  double Jphi;
  double L;
  int i;
  // function [validity,iter,rval,xmin]=dbrent_adj
  //  ---------------------------------------------------------------------------------%
  Jphi = 0.0;
  L = a + b;
  i = static_cast<int>(Nwp);
  for (int j{0}; j < i; j++) {
    double alpha;
    double b_a;
    double cos_psi;
    double d;
    // number of waypoints
    alpha = (x_wp[j] + b) * z / (L - y_wp[j] * z);
    // arctan(alpha)=psi_j circular approximation heading
    cos_psi = 1.0 / std::sqrt(alpha * alpha + 1.0);
    // sin_psi=alpha/sqrt(oneplusalsq);
    d = h_wp[j];
    b_a = std::sin(d) - alpha * cos_psi;
    alpha = std::cos(d) - cos_psi;
    Jphi += w[j] * (b_a * b_a + alpha * alpha);
  }
  // for
  return Jphi / Nwp;
}

//
// cost function for angular error - derivative
//  way point x_wp,y_wp, head_wp of dimension N
//  a+b=L vehicle wheel base
//  Th_jump=pi/2;angleprev=0.0;
//
// Arguments    : double z
//                const double x_wp[50]
//                const double y_wp[50]
//                const double h_wp[50]
//                const double w[50]
//                double a
//                double b
//                double Nwp
// Return Type  : double
//
static double Jadot(double z, const double x_wp[50], const double y_wp[50],
                    const double h_wp[50], const double w[50], double a,
                    double b, double Nwp)
{
  double Jphidot;
  double L;
  int i;
  // function
  //  ---------------------------- prior to 11/14/2016
  //  ----------------------------------------------% % function [cost_angle] =
  //  Ja(z,x_wp,y_wp,h_wp,w,a,b,Nwp) % %cost function for radial error % %   way
  //  point x_wp,y_wp, head_wp of dimension N % % a+b=L vehicle wheel base % %
  //  Th_jump=pi/2;angleprev=0.0; % Jphi=0.0; % L=a+b; % for j=1:Nwp %number of
  //  waypoints %     angle=-atan((x_wp(j)+b)*z/(y_wp(j)*z-L)); % %     if j==1
  //  % %         angleprev=angle;
  //  % %     end
  //  %
  //  % % 11/4/2016 assume that angles are unwrap, like asin(sin(xxx))
  //  % %     if j>1 %unwrap angle
  //  % %         if angle-angleprev>Th_jump
  //  % %             angle=angle-pi;
  //  % %         elseif angleprev-angle>Th_jump
  //  % %             angle=angle+pi;
  //  % %         end
  //  % %     end
  //  % %     Jphi=Jphi+w(j)*(angle-h_wp(j))^2;
  //  % %     angleprev=angle;
  //  % difference=h_wp(j)-angle;
  //  % Jphi=Jphi+w(j)*(asin(sin(difference)))^2;
  //  % end %for
  //  % cost_angle=Jphi/Nwp;
  //  % end %function
  //  ---------------------------------------------------------------------------------%
  Jphidot = 0.0;
  L = a + b;
  i = static_cast<int>(Nwp);
  for (int j{0}; j < i; j++) {
    double alpha;
    double alpha_tmp;
    double b_alpha_tmp;
    double cos_psi;
    double d;
    double oneplusalsq;
    // number of waypoints
    alpha_tmp = L - y_wp[j] * z;
    b_alpha_tmp = x_wp[j] + b;
    alpha = b_alpha_tmp * z / alpha_tmp;
    // arctan(alpha)=psi_j circular approximation heading
    oneplusalsq = alpha * alpha + 1.0;
    cos_psi = 1.0 / std::sqrt(oneplusalsq);
    // sin_psi=alpha/sqrt(oneplusalsq);
    d = h_wp[j];
    Jphidot +=
        w[j] *
        ((std::sin(d) - alpha * cos_psi) - alpha * (std::cos(d) - cos_psi)) *
        (L * b_alpha_tmp / (alpha_tmp * alpha_tmp)) /
        rt_powd_snf(oneplusalsq, 1.5);
  }
  // for
  return -2.0 * Jphidot / Nwp;
}

//
// cost function for radial error - derivative
//    way point x_wp,y_wp, head_wp of dimansion N
//  a+b=L vehicle wheelbase
//
// Arguments    : double z
//                const double x_wp[50]
//                const double y_wp[50]
//                const double w[50]
//                double a
//                double b
//                double Nwp
// Return Type  : double
//
static double Jydot(double z, const double x_wp[50], const double y_wp[50],
                    const double w[50], double a, double b, double Nwp)
{
  double Jdot;
  double L;
  int i;
  // function
  //  ---------------------------------------------------------------------------------%
  L = a + b;
  Jdot = 0.0;
  i = static_cast<int>(Nwp);
  for (int j{0}; j < i; j++) {
    double b_zden1_tmp;
    double d;
    double zden;
    double zden1;
    double zden1_tmp;
    double zden2;
    double zden2_tmp;
    double znum;
    double znum_tmp;
    // number of waypoints
    d = y_wp[j];
    zden = d * d;
    zden1 = 2.0 * d * L;
    zden2 = x_wp[j];
    znum_tmp = (zden2 * zden2 + 2.0 * b * zden2) + zden;
    znum = znum_tmp * z - zden1;
    zden2 += b;
    zden1_tmp = L * L;
    b_zden1_tmp = zden2 * zden2 + zden;
    zden1 = std::sqrt((b_zden1_tmp * (z * z) - zden1 * z) + zden1_tmp);
    zden2_tmp = b * b * z;
    zden2 = std::sqrt(zden1_tmp + zden2_tmp * z);
    zden = zden1 + zden2;
    //     Vdot=(((x_wp(j)+b)^2+y_wp(j)^2)*z-y_wp(j)*L-y_wp(j)*L)/zden1+b^2*z/zden2;
    //     %has error fixed on 11/15/2016
    Jdot += 2.0 * w[j] * (znum / zden) *
            (znum_tmp * zden -
             znum * ((b_zden1_tmp * z - d * L) / zden1 + zden2_tmp / zden2)) /
            zden / zden;
  }
  // for
  return Jdot / Nwp;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

//
// INPUTS
// MnvrActive, TimeInMnvr
//
//  3/17/2016 Pure Pursuit
//  xk=[dy,dpsi,tan(RWA)] (PA~HWA)
//  State_vec_new=[State_vec(1);State_vec(2);State_vec(3)];
//  control u is tangent of road wheel angle; u=tan(delta)
//
// Arguments    : double MnvrActive
//                double TimeInMnvr
//                double tanRWA
//                double tanRWA_prev
//                const double desPathX[50]
//                const double desPathY[50]
//                const double desPhi[50]
//                double PPursuit_tun1
//                double L_preview
//                const double veh_param[2]
//                double brent_tol
//                double brent_itmax
//                double brent_zeps
//                double wdist
//                double wangle
//                const double weight[50]
//                double method4ini
//                double *tandelta_NF
//                double *costfun_value
//                double *number_iter
//                double *brent_valid
//                double *j_preview
//                double *delta
//                double *tandelta_PP
//                double *cost
//                double *cost_deriv
//                double *Nwp1
//                double *bx
//                double *cost_dd0_2
//                double *tandelta_PP2
// Return Type  : void
//
void NonlinCircleFit(double MnvrActive, double TimeInMnvr, double tanRWA,
                     double tanRWA_prev, const double desPathX[50],
                     const double desPathY[50], const double desPhi[50],
                     double PPursuit_tun1, double L_preview,
                     const double veh_param[2], double brent_tol,
                     double brent_itmax, double brent_zeps, double wdist,
                     double wangle, const double weight[50], double method4ini,
                     double *tandelta_NF, double *costfun_value,
                     double *number_iter, double *brent_valid,
                     double *j_preview, double *delta, double *tandelta_PP,
                     double *cost, double *cost_deriv, double *Nwp1, double *bx,
                     double *cost_dd0_2, double *tandelta_PP2)
{
  double x_wp[50];
  double y_wp[50];
  double dL;
  double veh_L_tmp;
  double veh_a;
  double veh_b;
  *tandelta_NF = 0.0;
  *costfun_value = 0.0;
  *number_iter = 0.0;
  *brent_valid = 0.0;
  // iter_out=0;alpha=0.0;dist=0.0;
  *j_preview = 1.0;
  dL = 0.0;
  *delta = 0.0;
  *tandelta_PP = 0.0;
  *tandelta_PP2 = 0.0;
  *cost = 0.0;
  *cost_deriv = 0.0;
  *cost_dd0_2 = 0.1;
  *bx = tanRWA_prev;
  *Nwp1 = 1.0;
  //  Jaoptimum=0.0;Jyoptimum=0.0;
  //  Jydotoptimum=0.0;Jadotoptimum=0.0;
  //  x=0.0;y=0.0;phi=0.0;
  //  ARRAYSIZE = 50;      %% size of desired path array, max 2 sec of look
  //  ahead time _/
  // %%%% Check that size of LnCurvVector is greater than Nnew
  // !!!!!!!!!!!!!!!!!!! max(size(LnCurvVector))>=Nnew if not fill with zeros
  //  Ts=max(Ts,0.0001); %protection from division by zero
  //  nLookPts = max(2,floor(PPursuit_nLookPts));       %% number of points in
  //  desired path for look ahead time period Nwp=min(Nwp,50); %Nwp should not
  //  exceed size on wp vector
  // %%%% INPUT VARIBALES INITIALIZATION
  veh_a = veh_param[0];
  //  distance(CG,front)[m]
  veh_b = veh_param[1];
  //  distance(CG,rear)[m]
  veh_L_tmp = veh_param[0] + veh_param[1];
  //  M=veh_param(3);  % mass [kg]
  //  Iz=veh_param(4);   % vehicle inertia-yaw [kg*m^2]
  //  Cf=veh_param(5);
  //  Cr=veh_param(6);
  //  c=veh_param(7);
  //  n=veh_param(8);
  //  I=veh_param(9); %steering inertia
  //  Df=veh_param(10); %Nm/rad 10Nm of steering wheel torque per one degree of
  //  slip angle
  if ((MnvrActive == 1.0) && (TimeInMnvr > 0.01)) {
    double J;
    double Jdd;
    double Ud_0;
    double V_0;
    double a;
    double a_tmp;
    double b;
    double b_a;
    double d;
    double d1;
    double dv;
    double dw;
    double dx;
    double e;
    double fv;
    double fw;
    double tol1;
    double tol2;
    double u1;
    double v;
    double w;
    int angle1_tmp;
    int b_angle1_tmp;
    int iter;
    //  alpha=-desPhi(nLookPts);
    //  dist=sqrt(desPathX(nLookPts)^2+desPathY(nLookPts)^2);
    //  dist=max(dist,0.01);
    //  tandelta=-PPursuit_tun1*2*veh_L*sin(alpha)/dist;
    //  % x=desPathX(nLookPts);
    //  % y=desPathY(nLookPts);
    //  % phi=-desPhi(nLookPts);
    //  tanRWA=State_vec(3);
    //  Nwp=50;
    // 1e-3; %0.0001
    //  weight=ones(Nwp,1);
    //  weight=ones(50,1);
    //  bx=tanRWA;
    // bx=tanRWA_prev;
    //  ax=bx-0.1;cx=bx+0.1;
    std::copy(&desPathX[0], &desPathX[50], &x_wp[0]);
    std::copy(&desPathY[0], &desPathY[50], &y_wp[0]);
    //  preview distance is given, based on it we need to modify waypoints
    //  L_preview=5.0; %meters
    while ((dL < L_preview) && (*j_preview < 50.0)) {
      angle1_tmp = static_cast<int>(*j_preview);
      a = desPathX[angle1_tmp] - desPathX[angle1_tmp - 1];
      b_a = desPathY[angle1_tmp] - desPathY[angle1_tmp - 1];
      dL += std::sqrt(a * a + b_a * b_a);
      (*j_preview)++;
    }
    (*j_preview)--;
    //  if j_preview+1<50
    *delta = dL - L_preview;
    angle1_tmp = static_cast<int>(*j_preview);
    b_angle1_tmp = static_cast<int>(*j_preview) - 1;
    dL = std::atan((desPathY[angle1_tmp] - desPathY[b_angle1_tmp]) /
                   (desPathX[angle1_tmp] - desPathX[b_angle1_tmp]));
    angle1_tmp = static_cast<int>(*j_preview);
    x_wp[angle1_tmp] = desPathX[angle1_tmp] - *delta * std::cos(dL);
    // modifying last non-zero value
    y_wp[angle1_tmp] = desPathY[angle1_tmp] - *delta * std::sin(dL);
    b_angle1_tmp = 48 - angle1_tmp;
    if (b_angle1_tmp >= 0) {
      std::memset(&x_wp[angle1_tmp + 1], 0,
                  (((b_angle1_tmp + angle1_tmp) - angle1_tmp) + 1) *
                      sizeof(double));
      std::memset(&y_wp[angle1_tmp + 1], 0,
                  (((b_angle1_tmp + angle1_tmp) - angle1_tmp) + 1) *
                      sizeof(double));
    }
    *Nwp1 = std::fmin(40.0, *j_preview + 1.0);
    // hard coded upper cap = 15. Should be L_preview/waypoint_average_space+1
    //  Nwp1=j_preview+1;
    //  powerful method below 1/29/2018
    //  choosing initial point based on linearization of the cost function
    //  around zero - start
    // cost0=wdist*Jy(0.0,x_wp,y_wp,weight,veh_a,veh_b,Nwp1)+wangle*Ja(0.0,x_wp,y_wp,h_wp,weight,veh_a,veh_b,Nwp1);
    // zincr=0.01;
    // cost_dd0_1=(wdist*Jydot(zincr,x_wp,y_wp,weight,veh_a,veh_b,Nwp1)+wangle*Jadot(zincr,x_wp,y_wp,h_wp,weight,veh_a,veh_b,Nwp1)-cost_deriv0)/zincr;
    // cost function for radial error - second derivative at z=0 (for initial
    // search)
    //    way point x_wp,y_wp, head_wp of dimansion N
    //  a+b=L vehicle wheelbase
    // function
    //  ---------------------------------------------------------------------------------%
    Jdd = 0.0;
    b_angle1_tmp = static_cast<int>(*Nwp1);
    V_0 = 2.0 * veh_L_tmp;
    // for
    // analytical second derivative at zero
    // choose which one
    *bx = 0.0;
    // override above
    //  choosing initial point based on linearization of the cost function
    //  around zero - end
    // function [tandelta,nLookPts]=NonlinCircleFit
    //  =============== Functions used ===============%
    //  ---------------------------------------------------------------------------------%
    // | FUNCTION: dbrent
    // |
    // | PURPOSE: Given a function f and its derivative function df, and
    // |   given a bracketing triplet of abscissas ax, bx, cx [such that
    // |   bx is between ax and cx, and f(bx) is less than both f(ax) and
    // |   f(cx)], this routine isolates the minimum to a fractional precision
    // |   of about tol using a modification of Brent's method that uses
    // |   derivatives. The abscissa of the minimum is returned as xmin, and
    // |   the minimum function value is returned as rval, the returned
    // |   function value.
    // |
    // | REFERENCE:  Numerical recipes in C
    // |
    *brent_valid = 1.0;
    *number_iter = 0.0;
    // 100; %was 1000
    // 1.0e-6; % ZEPS  = 1.0e-10;
    e = 0.0;
    a = -1.0;
    b = 1.0;
    *tandelta_NF = 0.0;
    w = 0.0;
    v = 0.0;
    tol1 = veh_param[1];
    // cost function for radial error
    //    way point x_wp,y_wp, head_wp of dimansion N
    //  a+b=L vehicle wheel base
    // function
    //  % % ------------------------- prior to 11/14/2016
    //  --------------------------------------------%
    //  %
    //  % function [cost_angle_dot] = Jadot(z,x_wp,y_wp,h_wp,w,a,b,Nwp)
    //  % %cost function for angular error - derivative
    //  % %   way point x_wp,y_wp, head_wp of dimension N
    //  % % a+b=L vehicle wheel base
    //  % % Th_jump=pi/2;angleprev=0.0;
    //  % Jphidot=0.0;
    //  % L=a+b;
    //  % for j=1:Nwp %number of waypoints
    //  %
    //  %     angle=-atan((x_wp(j)+b)*z/(y_wp(j)*z-L));
    //  % %     if j==1 %unwrap angle
    //  % %         angleprev=angle;
    //  % %     end
    //  % %
    //  % %     if j>1 %unwrap angle
    //  % %         if angle-angleprev>Th_jump
    //  % %             angle=angle-pi;
    //  % %         elseif angleprev-angle>Th_jump
    //  % %             angle=angle+pi;
    //  % %         end
    //  % %     end
    //  %     difference=h_wp(j)-angle;
    //  %
    //  Jphidot=Jphidot+w(j)*(asin(sin(difference)))*(-(x_wp(j)+b)*L)/(((x_wp(j)+b)*z)^2+(y_wp(j)*z-L)^2);
    //  % %     angleprev=angle;
    //  % end %for
    //  % cost_angle_dot=2*Jphidot/Nwp;
    //  % end %function
    //  ---------------------------------------------------------------------------------%
    J = 0.0;
    tol2 = V_0 * V_0;
    a_tmp = veh_L_tmp * veh_L_tmp;
    for (int j{0}; j < b_angle1_tmp; j++) {
      // number of waypoints
      d = y_wp[j];
      dL = -2.0 * d * veh_L_tmp;
      d1 = x_wp[j];
      fw = d * d;
      fv = d1 * d1;
      Ud_0 = (fv + 2.0 * veh_b * d1) + fw;
      b_a = d1 + veh_b;
      u1 = dL * dL;
      dx = weight[j];
      Jdd += 2.0 * dx *
             ((((rt_powd_snf(V_0, 3.0) * (Ud_0 * Ud_0) -
                 4.0 * dL * tol2 * Ud_0 * -d) +
                3.0 * u1 * V_0 * (-d * -d)) -
               u1 * tol2 * ((b_a * b_a + veh_b * veh_b) / veh_L_tmp)) /
              rt_powd_snf(V_0, 5.0));
      //      znum=(x_wp(j)^2+2.0*b*x_wp(j)+y_wp(j)^2)*z-2.0*y_wp(j)*L;
      //      zden1=sqrt(((x_wp(j)+b)^2+y_wp(j)^2)*z^2-2.0*y_wp(j)*L*z+L^2);
      //      zden2=sqrt(L^2+b*b*z*z);
      //      zden=zden1+zden2;
      //      U=znum;V=zden;
      //      Udot=x_wp(j)^2+2.0*b*x_wp(j)+y_wp(j)^2;
      //      %
      //      Vdot=(((x_wp(j)+b)^2+y_wp(j)^2)*z-y_wp(j)*L-y_wp(j)*L)/zden1+b^2*z/zden2;
      //      %has error fixed on 11/15/2016
      //      Vdot=(((x_wp(j)+b)^2+y_wp(j)^2)*z-y_wp(j)*L)/zden1+b^2*z/zden2;
      //      Jdot=Jdot+2.0*w(j)*(znum/zden)*(Udot*V-U*Vdot)/V/V;
      // number of waypoints
      b_a = d1 + tol1;
      u1 = 2.0 * d * veh_L_tmp;
      b_a = (((fv + 2.0 * tol1 * d1) + fw) * 0.0 - u1) /
            (std::sqrt(((b_a * b_a + fw) * 0.0 - u1 * 0.0) + a_tmp) +
             std::sqrt(a_tmp + tol1 * tol1 * 0.0 * 0.0));
      J += dx * (b_a * b_a);
    }
    *cost_dd0_2 = Jdd / *Nwp1;
    // for
    *costfun_value =
        wdist * (J / *Nwp1) + wangle * Ja(0.0, x_wp, y_wp, desPhi, weight,
                                          veh_param[0], veh_param[1], *Nwp1);
    // fx = feval(f,x);
    fw = *costfun_value;
    fv = *costfun_value;
    dx = wdist *
             Jydot(0.0, x_wp, y_wp, weight, veh_param[0], veh_param[1], *Nwp1) +
         wangle * Jadot(0.0, x_wp, y_wp, desPhi, weight, veh_param[0],
                        veh_param[1], *Nwp1);
    // dx = feval(df,x);
    dw = dx;
    dv = dx;
    iter = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (iter <= static_cast<int>(brent_itmax) - 1) {
        (*number_iter)++;
        V_0 = 0.5 * (a + b);
        tol1 = brent_tol * std::abs(*tandelta_NF) + brent_zeps;
        tol2 = 2.0 * tol1;
        d = b - a;
        if (std::abs(*tandelta_NF - V_0) <= tol2 - 0.5 * d) {
          exitg1 = 1;
        } else {
          bool guard1;
          if (std::abs(e) > tol1) {
            bool ok1;
            bool ok2;
            dL = 2.0 * d;
            //  Initialize these d's to an out-of-bracket value
            Ud_0 = dL;
            if (dw != dx) {
              dL = (w - *tandelta_NF) * dx / (dx - dw);
              //  Secant method with one point.
            }
            if (dv != dx) {
              Ud_0 = (v - *tandelta_NF) * dx / (dx - dv);
              //  Secant method with the other point.
            }
            // |  Which of these two estimates of d shall we take? We will
            // |  insist that they be within the bracket, and on the side
            // |  pointed to by the derivative at x:
            u1 = *tandelta_NF + dL;
            Jdd = *tandelta_NF + Ud_0;
            ok1 = (((a - u1) * (u1 - b) > 0.0) && (dx * dL <= 0.0));
            ok2 = (((a - Jdd) * (Jdd - b) > 0.0) && (dx * Ud_0 <= 0.0));
            //  Movement on the step before last.
            //        e    = d; % - - - ?? Looks like this statement is not
            //        needed
            // |  Take only an acceptable d, and if both are acceptable,
            // |  then take the smallest one.
            if (ok1 || ok2) {
              if (ok1 && ok2) {
                if (!(std::abs(dL) < std::abs(Ud_0))) {
                  dL = Ud_0;
                }
              } else if (!ok1) {
                dL = Ud_0;
              }
              if (std::abs(dL) <= std::abs(0.5 * e)) {
                Jdd = *tandelta_NF + dL;
                if ((Jdd - a < tol2) || (b - Jdd < tol2)) {
                  // function
                  //  ---------------------------------------------------------------------------------%
                  // | nzSIGN  returns the abs() of the first argument if the
                  // 2nd argument is |    is >= 0.  Otherwise returnd the -abs()
                  // of the first argument.
                  // |
                  // | PURPOSE:  written to match "numerical recipes in C"
                  // function SIGN. |    Unlike matlab's 'sign', a 0.0 value
                  // does not force the result to 0.0
                  // |
                  if (V_0 - *tandelta_NF >= 0.0) {
                    dL = std::abs(tol1);
                  } else {
                    dL = -std::abs(tol1);
                  }
                }
              } else {
                //  Bisect, not golden section.
                // |  Decide which segment by the nzSIGN of the derivative.
                if (dx >= 0.0) {
                  e = a - *tandelta_NF;
                } else {
                  e = b - *tandelta_NF;
                }
                dL = 0.5 * e;
              }
            } else {
              if (dx >= 0.0) {
                e = a - *tandelta_NF;
              } else {
                e = b - *tandelta_NF;
              }
              dL = 0.5 * e;
            }
          } else {
            if (dx >= 0.0) {
              e = a - *tandelta_NF;
            } else {
              e = b - *tandelta_NF;
            }
            dL = 0.5 * e;
          }
          guard1 = false;
          if (std::abs(dL) >= tol1) {
            Jdd = *tandelta_NF + dL;
            // cost function for radial error
            //    way point x_wp,y_wp, head_wp of dimansion N
            //  a+b=L vehicle wheel base
            // function
            //  % % ------------------------- prior to 11/14/2016
            //  --------------------------------------------%
            //  %
            //  % function [cost_angle_dot] = Jadot(z,x_wp,y_wp,h_wp,w,a,b,Nwp)
            //  % %cost function for angular error - derivative
            //  % %   way point x_wp,y_wp, head_wp of dimension N
            //  % % a+b=L vehicle wheel base
            //  % % Th_jump=pi/2;angleprev=0.0;
            //  % Jphidot=0.0;
            //  % L=a+b;
            //  % for j=1:Nwp %number of waypoints
            //  %
            //  %     angle=-atan((x_wp(j)+b)*z/(y_wp(j)*z-L));
            //  % %     if j==1 %unwrap angle
            //  % %         angleprev=angle;
            //  % %     end
            //  % %
            //  % %     if j>1 %unwrap angle
            //  % %         if angle-angleprev>Th_jump
            //  % %             angle=angle-pi;
            //  % %         elseif angleprev-angle>Th_jump
            //  % %             angle=angle+pi;
            //  % %         end
            //  % %     end
            //  %     difference=h_wp(j)-angle;
            //  %
            //  Jphidot=Jphidot+w(j)*(asin(sin(difference)))*(-(x_wp(j)+b)*L)/(((x_wp(j)+b)*z)^2+(y_wp(j)*z-L)^2);
            //  % %     angleprev=angle;
            //  % end %for
            //  % cost_angle_dot=2*Jphidot/Nwp;
            //  % end %function
            //  ---------------------------------------------------------------------------------%
            J = 0.0;
            a_tmp = veh_L_tmp * veh_L_tmp;
            for (int j{0}; j < b_angle1_tmp; j++) {
              // number of waypoints
              d = x_wp[j];
              b_a = d + veh_b;
              d1 = y_wp[j];
              u1 = d1 * d1;
              dL = 2.0 * d1 * veh_L_tmp;
              b_a = (((d * d + 2.0 * veh_b * d) + u1) * Jdd - dL) /
                    (std::sqrt(((b_a * b_a + u1) * (Jdd * Jdd) - dL * Jdd) +
                               a_tmp) +
                     std::sqrt(a_tmp + veh_b * veh_b * Jdd * Jdd));
              J += weight[j] * (b_a * b_a);
            }
            // for
            Ud_0 =
                wdist * (J / *Nwp1) + wangle * Ja(Jdd, x_wp, y_wp, desPhi,
                                                  weight, veh_a, veh_b, *Nwp1);
            // fu = feval(f,u);
            guard1 = true;
          } else {
            // function
            //  ---------------------------------------------------------------------------------%
            // | nzSIGN  returns the abs() of the first argument if the 2nd
            // argument is |    is >= 0.  Otherwise returnd the -abs() of the
            // first argument.
            // |
            // | PURPOSE:  written to match "numerical recipes in C" function
            // SIGN. |    Unlike matlab's 'sign', a 0.0 value does not force the
            // result to 0.0
            // |
            if (dL >= 0.0) {
              dL = std::abs(tol1);
            } else {
              dL = -std::abs(tol1);
            }
            Jdd = *tandelta_NF + dL;
            // cost function for radial error
            //    way point x_wp,y_wp, head_wp of dimansion N
            //  a+b=L vehicle wheel base
            // function
            //  % % ------------------------- prior to 11/14/2016
            //  --------------------------------------------%
            //  %
            //  % function [cost_angle_dot] = Jadot(z,x_wp,y_wp,h_wp,w,a,b,Nwp)
            //  % %cost function for angular error - derivative
            //  % %   way point x_wp,y_wp, head_wp of dimension N
            //  % % a+b=L vehicle wheel base
            //  % % Th_jump=pi/2;angleprev=0.0;
            //  % Jphidot=0.0;
            //  % L=a+b;
            //  % for j=1:Nwp %number of waypoints
            //  %
            //  %     angle=-atan((x_wp(j)+b)*z/(y_wp(j)*z-L));
            //  % %     if j==1 %unwrap angle
            //  % %         angleprev=angle;
            //  % %     end
            //  % %
            //  % %     if j>1 %unwrap angle
            //  % %         if angle-angleprev>Th_jump
            //  % %             angle=angle-pi;
            //  % %         elseif angleprev-angle>Th_jump
            //  % %             angle=angle+pi;
            //  % %         end
            //  % %     end
            //  %     difference=h_wp(j)-angle;
            //  %
            //  Jphidot=Jphidot+w(j)*(asin(sin(difference)))*(-(x_wp(j)+b)*L)/(((x_wp(j)+b)*z)^2+(y_wp(j)*z-L)^2);
            //  % %     angleprev=angle;
            //  % end %for
            //  % cost_angle_dot=2*Jphidot/Nwp;
            //  % end %function
            //  ---------------------------------------------------------------------------------%
            J = 0.0;
            a_tmp = veh_L_tmp * veh_L_tmp;
            for (int j{0}; j < b_angle1_tmp; j++) {
              // number of waypoints
              d = x_wp[j];
              b_a = d + veh_b;
              d1 = y_wp[j];
              u1 = d1 * d1;
              dL = 2.0 * d1 * veh_L_tmp;
              b_a = (((d * d + 2.0 * veh_b * d) + u1) * Jdd - dL) /
                    (std::sqrt(((b_a * b_a + u1) * (Jdd * Jdd) - dL * Jdd) +
                               a_tmp) +
                     std::sqrt(a_tmp + veh_b * veh_b * Jdd * Jdd));
              J += weight[j] * (b_a * b_a);
            }
            // for
            Ud_0 =
                wdist * (J / *Nwp1) + wangle * Ja(Jdd, x_wp, y_wp, desPhi,
                                                  weight, veh_a, veh_b, *Nwp1);
            // fu = feval(f,u);
            // |  If the minimum step in the downhill direction takes us
            // |  uphill, then we are done.
            if (Ud_0 > *costfun_value) {
              exitg1 = 1;
            } else {
              guard1 = true;
            }
          }
          if (guard1) {
            // |  Housekeeping
            u1 = wdist * Jydot(Jdd, x_wp, y_wp, weight, veh_a, veh_b, *Nwp1) +
                 wangle * Jadot(Jdd, x_wp, y_wp, desPhi, weight, veh_a, veh_b,
                                *Nwp1);
            // du = feval(df, u);
            if (Ud_0 <= *costfun_value) {
              if (Jdd >= *tandelta_NF) {
                a = *tandelta_NF;
              } else {
                b = *tandelta_NF;
              }
              v = w;
              fv = fw;
              dv = dw;
              w = *tandelta_NF;
              fw = *costfun_value;
              dw = dx;
              *tandelta_NF = Jdd;
              *costfun_value = Ud_0;
              dx = u1;
            } else {
              if (Jdd < *tandelta_NF) {
                a = Jdd;
              } else {
                b = Jdd;
              }
              if ((Ud_0 <= fw) || (w == *tandelta_NF)) {
                v = w;
                fv = fw;
                dv = dw;
                w = Jdd;
                fw = Ud_0;
                dw = u1;
              } else if ((Ud_0 < fv) || (v == *tandelta_NF) || (v == w)) {
                v = Jdd;
                fv = Ud_0;
                dv = u1;
              }
            }
            iter++;
          }
        }
      } else {
        //  disp('Too many iterations in routine dbrent');
        *brent_valid = 0.0;
        //  hopefully never get here.
        exitg1 = 1;
      }
    } while (exitg1 == 0);
    //  out1=xmin;  %xmin is optimal tandelta
    // iter_out;
    //  if validity==1
    //  end
    dL = x_wp[angle1_tmp];
    Ud_0 = y_wp[angle1_tmp];
    u1 = dL * dL + Ud_0 * Ud_0;
    *tandelta_PP = PPursuit_tun1 * 2.0 * veh_L_tmp * Ud_0 / u1;
    // classic formula
    *tandelta_PP2 = 2.0 * veh_L_tmp * Ud_0 / (u1 + 2.0 * dL * veh_param[1]);
    // circle will pass through point (j_preview+1)
    //  Jyoptimum=Jy(xmin,x_wp,y_wp,weight,veh_a,veh_b,Nwp1);
    //  Jaoptimum=Ja(xmin,x_wp,y_wp,h_wp,weight,veh_a,veh_b,Nwp1);
    //  Jydotoptimum=Jydot(xmin,x_wp,y_wp,weight,veh_a,veh_b,Nwp1);
    //  Jadotoptimum=Jadot(xmin,x_wp,y_wp,h_wp,weight,veh_a,veh_b,Nwp1);
    b = veh_param[1];
    // cost function for radial error
    //    way point x_wp,y_wp, head_wp of dimansion N
    //  a+b=L vehicle wheel base
    // function
    //  % % ------------------------- prior to 11/14/2016
    //  --------------------------------------------%
    //  %
    //  % function [cost_angle_dot] = Jadot(z,x_wp,y_wp,h_wp,w,a,b,Nwp)
    //  % %cost function for angular error - derivative
    //  % %   way point x_wp,y_wp, head_wp of dimension N
    //  % % a+b=L vehicle wheel base
    //  % % Th_jump=pi/2;angleprev=0.0;
    //  % Jphidot=0.0;
    //  % L=a+b;
    //  % for j=1:Nwp %number of waypoints
    //  %
    //  %     angle=-atan((x_wp(j)+b)*z/(y_wp(j)*z-L));
    //  % %     if j==1 %unwrap angle
    //  % %         angleprev=angle;
    //  % %     end
    //  % %
    //  % %     if j>1 %unwrap angle
    //  % %         if angle-angleprev>Th_jump
    //  % %             angle=angle-pi;
    //  % %         elseif angleprev-angle>Th_jump
    //  % %             angle=angle+pi;
    //  % %         end
    //  % %     end
    //  %     difference=h_wp(j)-angle;
    //  %
    //  Jphidot=Jphidot+w(j)*(asin(sin(difference)))*(-(x_wp(j)+b)*L)/(((x_wp(j)+b)*z)^2+(y_wp(j)*z-L)^2);
    //  % %     angleprev=angle;
    //  % end %for
    //  % cost_angle_dot=2*Jphidot/Nwp;
    //  % end %function
    //  ---------------------------------------------------------------------------------%
    J = 0.0;
    a_tmp = veh_L_tmp * veh_L_tmp;
    for (int j{0}; j < b_angle1_tmp; j++) {
      // number of waypoints
      d = x_wp[j];
      a = d + b;
      d1 = y_wp[j];
      u1 = d1 * d1;
      dL = 2.0 * d1 * veh_L_tmp;
      a = (((d * d + 2.0 * b * d) + u1) * *tandelta_NF - dL) /
          (std::sqrt(((a * a + u1) * (*tandelta_NF * *tandelta_NF) -
                      dL * *tandelta_NF) +
                     a_tmp) +
           std::sqrt(a_tmp + b * b * *tandelta_NF * *tandelta_NF));
      J += weight[j] * (a * a);
    }
    // for
    *cost = wdist * (J / *Nwp1) + wangle * Ja(*tandelta_NF, x_wp, y_wp, desPhi,
                                              weight, veh_param[0],
                                              veh_param[1], *Nwp1);
    *cost_deriv = wdist * Jydot(*tandelta_NF, x_wp, y_wp, weight, veh_param[0],
                                veh_param[1], *Nwp1) +
                  wangle * Jadot(*tandelta_NF, x_wp, y_wp, desPhi, weight,
                                 veh_param[0], veh_param[1], *Nwp1);
  }
  // if MnvrActive==1&&TimeInMnvr>0.01
}

//
// File trailer for NonlinCircleFit.cpp
//
// [EOF]
//
