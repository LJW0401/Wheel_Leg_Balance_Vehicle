/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * LegSpd.c
 *
 * Code generation for function 'LegSpd'
 *
 */

/* Include files */
#include "LegSpd.h"
#include <math.h>

/* Function Definitions */
  /* LegSpd */
  /*     SPD = LegSpd(DPHI1,DPHI4,PHI1,PHI4) */
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     2023-10-04 14:10:26 */
void LegSpd(float dphi1, float dphi4, float phi1, float phi4, float spd[2])
{
  float t10_tmp;
  float t12_tmp;
  float t2;
  float t21;
  float t22;
  float t23;
  float t24;
  float t28;
  float t3;
  float t30;
  float t31;
  float t38;
  float t4;
  float t44;
  float t47;
  float t48;
  float t5;
  float t52;
  float t53;
  float t59;
  float t60;
  float t70;
  float t71;
  float t76;

  t2 = cos(phi1);
  t3 = cos(phi4);
  t4 = sin(phi1);
  t5 = sin(phi4);
  t10_tmp = t2 * 0.095;
  t12_tmp = t4 * 0.095;
  t21 = t2 * 0.0361;
  t22 = t3 * 0.0361;
  t23 = t4 * 0.0361;
  t24 = t5 * 0.0361;
  t28 = t12_tmp - t5 * 0.095;
  t30 = (t3 * 0.095 - t10_tmp) + 0.095;
  t31 = t23 - t24;
  t38 = (t22 - t21) + 0.0361;
  t44 = t28 * t28 + t30 * t30;
  t47 = t2 * t28 * 0.19 + t4 * t30 * 0.19;
  t48 = t3 * t28 * 0.19 + t5 * t30 * 0.19;
  t52 = 1.0 / (t38 + t44);
  t53 = t52 * t52;
  t59 = sqrt((t31 * t31 + t38 * t38) - t44 * t44);
  t60 = 1.0 / t59;
  t30 = (t24 - t23) + t59;
  t28 = atan(t52 * t30) * 2.0;
  t70 = cos(t28);
  t71 = sin(t28);
  t76 = 1.0 / (t53 * (t30 * t30) + 1.0);
  t47 =
      (t23 + t47) * t53 * t30 +
      t52 * (t21 -
             t60 * ((t2 * t31 * 0.0722 + t4 * t38 * 0.0722) - t44 * t47 * 2.0) /
                 2.0);
  t28 =
      (t24 + t48) * t53 * t30 +
      t52 * (t22 -
             t60 * ((t3 * t31 * 0.0722 + t5 * t38 * 0.0722) - t44 * t48 * 2.0) /
                 2.0);
  t4 = t12_tmp + t71 * 0.19;
  t21 = (t10_tmp + t70 * 0.19) - 0.0475;
  t59 = t70 * t76;
  t23 = t59 * t28;
  t53 = t71 * t76;
  t2 = t53 * t28;
  t28 = (-t10_tmp - t70 * 0.19) + 0.0475;
  t60 = t28 * t28;
  t52 = 1.0 / t28;
  t30 = t4 * t4;
  t28 = 1.0 / sqrt(t30 + t21 * t21);
  t48 = 1.0 / (t30 + t60);
  t59 = t10_tmp - t59 * t47 * 0.38;
  t30 = t12_tmp - t53 * t47 * 0.38;
  spd[0] = dphi4 * t28 * (t4 * t23 * 0.76 - t21 * t2 * 0.76) / 2.0 +
           dphi1 * t28 * (t4 * t59 * 2.0 - t21 * t30 * 2.0) / 2.0;
  t28 = t4 * (1.0 / t60);
  spd[1] = -dphi1 * t60 * t48 * (t52 * t59 - t28 * t30) +
           dphi4 * t60 * t48 * (t52 * (0.0 - t23 * 0.38) + t28 * (t2 * 0.38));
}

/* End of code generation (LegSpd.c) */
