/*
 * File: Inverse_kinematics_v1.c
 *
 * MATLAB Coder version            : 4.3
 * C/C++ source code generated on  : 03-Apr-2020 11:55:34
 */

/* Include Files */
#include "Inverse_kinematics_v1.h"
#include "Inverse_kinematics.h"
#include "Inverse_kinematics_v1_data.h"
#include "Inverse_kinematics_v1_initialize.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */

/*
 * Arguments    : const double system_parameters[5]
 *                const double phi_ee[3]
 *                double q_v1[3]
 * Return Type  : void
 */
void Inverse_kinematics_v1(const double system_parameters[5], const double
  phi_ee[3], double q_v1[3])
{
  double t2_tmp;
  double t3_tmp;
  double t4_tmp;
  double t6_tmp;
  double t7_tmp;
  double t8_tmp;
  double t9_tmp;
  double t10_tmp;
  double t11_tmp;
  double t13_tmp;
  double t14_tmp;
  double t15_tmp;
  double q_v1_tmp_tmp;
  double b_q_v1_tmp_tmp;
  double c_q_v1_tmp_tmp;
  double d_q_v1_tmp_tmp;
  double e_q_v1_tmp_tmp;
  double q_v1_tmp;
  double b_q_v1_tmp;
  double c_q_v1_tmp;
  double d_q_v1_tmp;
  double f_q_v1_tmp_tmp;
  double e_q_v1_tmp;
  double f_q_v1_tmp;
  double g_q_v1_tmp_tmp;
  double h_q_v1_tmp_tmp;
  double g_q_v1_tmp;
  double i_q_v1_tmp_tmp;
  double h_q_v1_tmp;
  double q_v1_tmp_tmp_tmp;
  double j_q_v1_tmp_tmp;
  double k_q_v1_tmp_tmp;
  double l_q_v1_tmp_tmp;
  double i_q_v1_tmp;
  double j_q_v1_tmp;
  double k_q_v1_tmp;
  double l_q_v1_tmp;
  double m_q_v1_tmp;
  double n_q_v1_tmp;
  double o_q_v1_tmp;
  double p_q_v1_tmp;
  double q_q_v1_tmp;
  double r_q_v1_tmp;
  double s_q_v1_tmp;
  double t_q_v1_tmp;
  if (isInitialized_Inverse_kinematics_v1 == false) {
    Inverse_kinematics_v1_initialize();
  }

  t2_tmp = cos(system_parameters[2]);
  t3_tmp = cos(system_parameters[0]);
  t4_tmp = cos(system_parameters[1]);
  t6_tmp = cos(phi_ee[0]);
  t7_tmp = cos(phi_ee[1]);
  t8_tmp = cos(phi_ee[2]);
  t9_tmp = sin(system_parameters[2]);
  t10_tmp = sin(system_parameters[0]);
  t11_tmp = sin(system_parameters[1]);
  t13_tmp = sin(phi_ee[0]);
  t14_tmp = sin(phi_ee[1]);
  t15_tmp = sin(phi_ee[2]);

  /*  Step 1: Find the theta angle without translation part */
  /*  Compute each angles of joints in legs in PSM */
  /*  phi_ee=[phi_1; phi_2; phi_3] - Angles in XYZ Euler angles parametrization */
  /*  system_parameters=[beta_1; beta_2; alpha_1; alpha_2, system_radius] - Parameters of PSM */
  /*  Created by Valeria Skvo */
  /* A_Q1 */
  /*     A_THETA = A_Q1(ETA_I,IN2,IN3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.4. */
  /*     31-Mar-2020 18:21:33 */
  /* B_Q1 */
  /*     B_THETA = B_Q1(ETA_I,IN2,IN3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.4. */
  /*     31-Mar-2020 18:21:34 */
  /* C_Q1 */
  /*     C_THETA = C_Q1(ETA_I,IN2,IN3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.4. */
  /*     31-Mar-2020 18:21:35 */
  q_v1_tmp_tmp = t2_tmp * t4_tmp;
  b_q_v1_tmp_tmp = t3_tmp * t4_tmp;
  c_q_v1_tmp_tmp = t2_tmp * t3_tmp;
  d_q_v1_tmp_tmp = t2_tmp * t7_tmp * t8_tmp * t10_tmp * t11_tmp;
  e_q_v1_tmp_tmp = t3_tmp * t7_tmp * t8_tmp * t9_tmp * t11_tmp;
  q_v1_tmp = t6_tmp * t11_tmp * t15_tmp;
  b_q_v1_tmp = t8_tmp * t11_tmp * t13_tmp * t14_tmp;
  c_q_v1_tmp = -cos(system_parameters[3]);
  d_q_v1_tmp = q_v1_tmp_tmp * t10_tmp;
  f_q_v1_tmp_tmp = t4_tmp * t6_tmp * t7_tmp * t9_tmp * t10_tmp;
  e_q_v1_tmp = (((c_q_v1_tmp - c_q_v1_tmp_tmp * t4_tmp * t6_tmp * t7_tmp) -
                 f_q_v1_tmp_tmp) + d_q_v1_tmp_tmp) - e_q_v1_tmp_tmp;
  f_q_v1_tmp = b_q_v1_tmp_tmp * t9_tmp;
  g_q_v1_tmp_tmp = t2_tmp * t6_tmp * t8_tmp * t10_tmp * t11_tmp;
  h_q_v1_tmp_tmp = t3_tmp * t6_tmp * t8_tmp * t9_tmp * t11_tmp;
  g_q_v1_tmp = c_q_v1_tmp_tmp * t11_tmp;
  i_q_v1_tmp_tmp = t9_tmp * t10_tmp * t11_tmp;
  h_q_v1_tmp = c_q_v1_tmp_tmp * t6_tmp * t8_tmp * t11_tmp;
  q_v1_tmp_tmp_tmp = t6_tmp * t8_tmp;
  j_q_v1_tmp_tmp = q_v1_tmp_tmp_tmp * t9_tmp * t10_tmp * t11_tmp;
  k_q_v1_tmp_tmp = t2_tmp * t10_tmp * t11_tmp * t13_tmp * t14_tmp * t15_tmp;
  l_q_v1_tmp_tmp = t3_tmp * t9_tmp * t11_tmp * t13_tmp * t14_tmp * t15_tmp;
  i_q_v1_tmp = t4_tmp * t7_tmp;
  j_q_v1_tmp = t7_tmp * t11_tmp * t15_tmp;
  k_q_v1_tmp = q_v1_tmp_tmp * t10_tmp;
  c_q_v1_tmp = (((c_q_v1_tmp - c_q_v1_tmp_tmp * t4_tmp * t6_tmp * t7_tmp) +
                 f_q_v1_tmp_tmp) + d_q_v1_tmp_tmp) + e_q_v1_tmp_tmp;
  l_q_v1_tmp = b_q_v1_tmp_tmp * t9_tmp;
  m_q_v1_tmp = c_q_v1_tmp_tmp * t11_tmp;
  n_q_v1_tmp = c_q_v1_tmp_tmp * t6_tmp * t8_tmp * t11_tmp;
  o_q_v1_tmp = t8_tmp * t9_tmp * t10_tmp * t11_tmp * t13_tmp;
  p_q_v1_tmp = t6_tmp * t9_tmp * t10_tmp * t11_tmp * t14_tmp * t15_tmp;
  q_v1[0] = Inverse_kinematic_solver((((((((((((((((((((((((e_q_v1_tmp +
    d_q_v1_tmp * 0.0 * t14_tmp) - f_q_v1_tmp * 0.0 * t14_tmp) - q_v1_tmp_tmp *
    t7_tmp * t10_tmp * t13_tmp) + b_q_v1_tmp_tmp * t7_tmp * t9_tmp * t13_tmp) -
    c_q_v1_tmp_tmp * t8_tmp * t11_tmp * t13_tmp) + g_q_v1_tmp_tmp) -
    h_q_v1_tmp_tmp) - d_q_v1_tmp_tmp) + e_q_v1_tmp_tmp) - g_q_v1_tmp * 0.0 *
    t13_tmp * t15_tmp) - o_q_v1_tmp) - i_q_v1_tmp_tmp * 0.0 * t13_tmp * t15_tmp)
    - c_q_v1_tmp_tmp * t6_tmp * t11_tmp * t14_tmp * t15_tmp) + h_q_v1_tmp * 0.0 *
    t14_tmp) + t2_tmp * t6_tmp * t10_tmp * t11_tmp * 0.0 * t15_tmp) - t3_tmp *
    t6_tmp * t9_tmp * t11_tmp * 0.0 * t15_tmp) - t2_tmp * t7_tmp * t10_tmp *
    t11_tmp * 0.0 * t15_tmp) + t3_tmp * t7_tmp * t9_tmp * t11_tmp * 0.0 *
    t15_tmp) - p_q_v1_tmp) + j_q_v1_tmp_tmp * 0.0 * t14_tmp) - k_q_v1_tmp_tmp) +
    l_q_v1_tmp_tmp) + t2_tmp * t8_tmp * t10_tmp * t11_tmp * 0.0 * t13_tmp *
    t14_tmp) - t3_tmp * t8_tmp * t9_tmp * t11_tmp * 0.0 * t13_tmp * t14_tmp,
    t9_tmp * (((((((((t4_tmp * t14_tmp - q_v1_tmp) + i_q_v1_tmp * 0.0 * t13_tmp)
                    - b_q_v1_tmp) + q_v1_tmp) - j_q_v1_tmp) - q_v1_tmp_tmp_tmp *
                 t11_tmp * 0.0) + t7_tmp * t8_tmp * t11_tmp * 0.0) + b_q_v1_tmp)
              + t11_tmp * 0.0 * t13_tmp * t14_tmp * t15_tmp) * 2.0,
    (((((((((((((((((((((((c_q_v1_tmp + k_q_v1_tmp * 0.0 * t14_tmp) + l_q_v1_tmp
    * 0.0 * t14_tmp) - q_v1_tmp_tmp * t7_tmp * t10_tmp * t13_tmp) -
    b_q_v1_tmp_tmp * t7_tmp * t9_tmp * t13_tmp) - c_q_v1_tmp_tmp * t8_tmp *
    t11_tmp * t13_tmp) + g_q_v1_tmp_tmp) + h_q_v1_tmp_tmp) - d_q_v1_tmp_tmp) -
                   e_q_v1_tmp_tmp) - m_q_v1_tmp * 0.0 * t13_tmp * t15_tmp) +
                 o_q_v1_tmp) + i_q_v1_tmp_tmp * 0.0 * t13_tmp * t15_tmp) -
               c_q_v1_tmp_tmp * t6_tmp * t11_tmp * t14_tmp * t15_tmp) +
              n_q_v1_tmp * 0.0 * t14_tmp) + t2_tmp * t6_tmp * t10_tmp * t11_tmp *
             0.0 * t15_tmp) + t3_tmp * t6_tmp * t9_tmp * t11_tmp * 0.0 * t15_tmp)
           - t2_tmp * t7_tmp * t10_tmp * t11_tmp * 0.0 * t15_tmp) - t3_tmp *
          t7_tmp * t9_tmp * t11_tmp * 0.0 * t15_tmp) + p_q_v1_tmp) -
        j_q_v1_tmp_tmp * 0.0 * t14_tmp) - k_q_v1_tmp_tmp) - l_q_v1_tmp_tmp) +
     t2_tmp * t8_tmp * t10_tmp * t11_tmp * 0.0 * t13_tmp * t14_tmp) + t3_tmp *
    t8_tmp * t9_tmp * t11_tmp * 0.0 * t13_tmp * t14_tmp, 0.0);

  /*  Step 1: Find the theta angle without translation part */
  /*  Compute each angles of joints in legs in PSM */
  /*  phi_ee=[phi_1; phi_2; phi_3] - Angles in XYZ Euler angles parametrization */
  /*  system_parameters=[beta_1; beta_2; alpha_1; alpha_2, system_radius] - Parameters of PSM */
  /*  Created by Valeria Skvo */
  /* A_Q1 */
  /*     A_THETA = A_Q1(ETA_I,IN2,IN3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.4. */
  /*     31-Mar-2020 18:21:33 */
  /* B_Q1 */
  /*     B_THETA = B_Q1(ETA_I,IN2,IN3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.4. */
  /*     31-Mar-2020 18:21:34 */
  /* C_Q1 */
  /*     C_THETA = C_Q1(ETA_I,IN2,IN3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.4. */
  /*     31-Mar-2020 18:21:35 */
  o_q_v1_tmp = c_q_v1_tmp_tmp * -0.49999999999999978;
  p_q_v1_tmp = t2_tmp * -0.49999999999999978;
  f_q_v1_tmp_tmp = t3_tmp * -0.49999999999999978;
  q_v1_tmp_tmp_tmp = c_q_v1_tmp_tmp * -0.49999999999999978;
  q_q_v1_tmp = t2_tmp * -0.49999999999999978;
  r_q_v1_tmp = t3_tmp * -0.49999999999999978;
  s_q_v1_tmp = -0.49999999999999978 * t8_tmp * t9_tmp * t10_tmp * t11_tmp *
    t13_tmp;
  t_q_v1_tmp = -0.49999999999999978 * t6_tmp * t9_tmp * t10_tmp * t11_tmp *
    t14_tmp * t15_tmp;
  q_v1[1] = Inverse_kinematic_solver((((((((((((((((((((((((e_q_v1_tmp +
    d_q_v1_tmp * 0.86602540378443871 * t14_tmp) - f_q_v1_tmp *
    0.86602540378443871 * t14_tmp) - q_v1_tmp_tmp * -0.49999999999999978 *
    t7_tmp * t10_tmp * t13_tmp) + b_q_v1_tmp_tmp * -0.49999999999999978 * t7_tmp
    * t9_tmp * t13_tmp) - o_q_v1_tmp * t8_tmp * t11_tmp * t13_tmp) +
    g_q_v1_tmp_tmp * 0.24999999999999978) - h_q_v1_tmp_tmp * 0.24999999999999978)
    - d_q_v1_tmp_tmp * 0.24999999999999978) + e_q_v1_tmp_tmp *
    0.24999999999999978) - g_q_v1_tmp * 0.86602540378443871 * t13_tmp * t15_tmp)
    - s_q_v1_tmp) - i_q_v1_tmp_tmp * 0.86602540378443871 * t13_tmp * t15_tmp) -
    o_q_v1_tmp * t6_tmp * t11_tmp * t14_tmp * t15_tmp) + h_q_v1_tmp *
    0.86602540378443871 * t14_tmp) + p_q_v1_tmp * t6_tmp * t10_tmp * t11_tmp *
    0.86602540378443871 * t15_tmp) - f_q_v1_tmp_tmp * t6_tmp * t9_tmp * t11_tmp *
    0.86602540378443871 * t15_tmp) - p_q_v1_tmp * t7_tmp * t10_tmp * t11_tmp *
    0.86602540378443871 * t15_tmp) + f_q_v1_tmp_tmp * t7_tmp * t9_tmp * t11_tmp *
    0.86602540378443871 * t15_tmp) - t_q_v1_tmp) + j_q_v1_tmp_tmp *
    0.86602540378443871 * t14_tmp) - k_q_v1_tmp_tmp * 0.24999999999999978) +
    l_q_v1_tmp_tmp * 0.24999999999999978) + p_q_v1_tmp * t8_tmp * t10_tmp *
    t11_tmp * 0.86602540378443871 * t13_tmp * t14_tmp) - f_q_v1_tmp_tmp * t8_tmp
    * t9_tmp * t11_tmp * 0.86602540378443871 * t13_tmp * t14_tmp, t9_tmp *
    (((((((((t4_tmp * -0.49999999999999978 * t14_tmp - q_v1_tmp) + i_q_v1_tmp *
            0.86602540378443871 * t13_tmp) - b_q_v1_tmp) + q_v1_tmp *
          0.24999999999999978) - j_q_v1_tmp * 0.24999999999999978) -
        -0.49999999999999978 * t6_tmp * t8_tmp * t11_tmp * 0.86602540378443871)
       + -0.49999999999999978 * t7_tmp * t8_tmp * t11_tmp * 0.86602540378443871)
      + b_q_v1_tmp * 0.24999999999999978) + -0.49999999999999978 * t11_tmp *
     0.86602540378443871 * t13_tmp * t14_tmp * t15_tmp) * 2.0,
    (((((((((((((((((((((((c_q_v1_tmp + k_q_v1_tmp * 0.86602540378443871 *
    t14_tmp) + l_q_v1_tmp * 0.86602540378443871 * t14_tmp) - q_v1_tmp_tmp *
    -0.49999999999999978 * t7_tmp * t10_tmp * t13_tmp) - b_q_v1_tmp_tmp *
    -0.49999999999999978 * t7_tmp * t9_tmp * t13_tmp) - q_v1_tmp_tmp_tmp *
    t8_tmp * t11_tmp * t13_tmp) + g_q_v1_tmp_tmp * 0.24999999999999978) +
                     h_q_v1_tmp_tmp * 0.24999999999999978) - d_q_v1_tmp_tmp *
                    0.24999999999999978) - e_q_v1_tmp_tmp * 0.24999999999999978)
                  - m_q_v1_tmp * 0.86602540378443871 * t13_tmp * t15_tmp) +
                 s_q_v1_tmp) + i_q_v1_tmp_tmp * 0.86602540378443871 * t13_tmp *
                t15_tmp) - q_v1_tmp_tmp_tmp * t6_tmp * t11_tmp * t14_tmp *
               t15_tmp) + n_q_v1_tmp * 0.86602540378443871 * t14_tmp) +
             q_q_v1_tmp * t6_tmp * t10_tmp * t11_tmp * 0.86602540378443871 *
             t15_tmp) + r_q_v1_tmp * t6_tmp * t9_tmp * t11_tmp *
            0.86602540378443871 * t15_tmp) - q_q_v1_tmp * t7_tmp * t10_tmp *
           t11_tmp * 0.86602540378443871 * t15_tmp) - r_q_v1_tmp * t7_tmp *
          t9_tmp * t11_tmp * 0.86602540378443871 * t15_tmp) + t_q_v1_tmp) -
        j_q_v1_tmp_tmp * 0.86602540378443871 * t14_tmp) - k_q_v1_tmp_tmp *
       0.24999999999999978) - l_q_v1_tmp_tmp * 0.24999999999999978) + q_q_v1_tmp
     * t8_tmp * t10_tmp * t11_tmp * 0.86602540378443871 * t13_tmp * t14_tmp) +
    r_q_v1_tmp * t8_tmp * t9_tmp * t11_tmp * 0.86602540378443871 * t13_tmp *
    t14_tmp, 0.0);

  /*  Step 1: Find the theta angle without translation part */
  /*  Compute each angles of joints in legs in PSM */
  /*  phi_ee=[phi_1; phi_2; phi_3] - Angles in XYZ Euler angles parametrization */
  /*  system_parameters=[beta_1; beta_2; alpha_1; alpha_2, system_radius] - Parameters of PSM */
  /*  Created by Valeria Skvo */
  /* A_Q1 */
  /*     A_THETA = A_Q1(ETA_I,IN2,IN3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.4. */
  /*     31-Mar-2020 18:21:33 */
  /* B_Q1 */
  /*     B_THETA = B_Q1(ETA_I,IN2,IN3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.4. */
  /*     31-Mar-2020 18:21:34 */
  /* C_Q1 */
  /*     C_THETA = C_Q1(ETA_I,IN2,IN3) */
  /*     This function was generated by the Symbolic Math Toolbox version 8.4. */
  /*     31-Mar-2020 18:21:35 */
  o_q_v1_tmp = c_q_v1_tmp_tmp * -0.50000000000000044;
  p_q_v1_tmp = t2_tmp * -0.50000000000000044;
  f_q_v1_tmp_tmp = t3_tmp * -0.50000000000000044;
  q_v1_tmp_tmp_tmp = c_q_v1_tmp_tmp * -0.50000000000000044;
  q_q_v1_tmp = t2_tmp * -0.50000000000000044;
  r_q_v1_tmp = t3_tmp * -0.50000000000000044;
  s_q_v1_tmp = -0.50000000000000044 * t8_tmp * t9_tmp * t10_tmp * t11_tmp *
    t13_tmp;
  t_q_v1_tmp = -0.50000000000000044 * t6_tmp * t9_tmp * t10_tmp * t11_tmp *
    t14_tmp * t15_tmp;
  q_v1[2] = Inverse_kinematic_solver((((((((((((((((((((((((e_q_v1_tmp +
    d_q_v1_tmp * -0.86602540378443849 * t14_tmp) - f_q_v1_tmp *
    -0.86602540378443849 * t14_tmp) - q_v1_tmp_tmp * -0.50000000000000044 *
    t7_tmp * t10_tmp * t13_tmp) + b_q_v1_tmp_tmp * -0.50000000000000044 * t7_tmp
    * t9_tmp * t13_tmp) - o_q_v1_tmp * t8_tmp * t11_tmp * t13_tmp) +
    g_q_v1_tmp_tmp * 0.25000000000000044) - h_q_v1_tmp_tmp * 0.25000000000000044)
    - d_q_v1_tmp_tmp * 0.25000000000000044) + e_q_v1_tmp_tmp *
    0.25000000000000044) - g_q_v1_tmp * -0.86602540378443849 * t13_tmp * t15_tmp)
    - s_q_v1_tmp) - i_q_v1_tmp_tmp * -0.86602540378443849 * t13_tmp * t15_tmp) -
    o_q_v1_tmp * t6_tmp * t11_tmp * t14_tmp * t15_tmp) + h_q_v1_tmp *
    -0.86602540378443849 * t14_tmp) + p_q_v1_tmp * t6_tmp * t10_tmp * t11_tmp *
    -0.86602540378443849 * t15_tmp) - f_q_v1_tmp_tmp * t6_tmp * t9_tmp * t11_tmp
    * -0.86602540378443849 * t15_tmp) - p_q_v1_tmp * t7_tmp * t10_tmp * t11_tmp *
    -0.86602540378443849 * t15_tmp) + f_q_v1_tmp_tmp * t7_tmp * t9_tmp * t11_tmp
    * -0.86602540378443849 * t15_tmp) - t_q_v1_tmp) + j_q_v1_tmp_tmp *
    -0.86602540378443849 * t14_tmp) - k_q_v1_tmp_tmp * 0.25000000000000044) +
    l_q_v1_tmp_tmp * 0.25000000000000044) + p_q_v1_tmp * t8_tmp * t10_tmp *
    t11_tmp * -0.86602540378443849 * t13_tmp * t14_tmp) - f_q_v1_tmp_tmp *
    t8_tmp * t9_tmp * t11_tmp * -0.86602540378443849 * t13_tmp * t14_tmp, t9_tmp
    * (((((((((t4_tmp * -0.50000000000000044 * t14_tmp - q_v1_tmp) + i_q_v1_tmp *
              -0.86602540378443849 * t13_tmp) - b_q_v1_tmp) + q_v1_tmp *
            0.25000000000000044) - j_q_v1_tmp * 0.25000000000000044) -
          -0.50000000000000044 * t6_tmp * t8_tmp * t11_tmp *
          -0.86602540378443849) + -0.50000000000000044 * t7_tmp * t8_tmp *
         t11_tmp * -0.86602540378443849) + b_q_v1_tmp * 0.25000000000000044) +
       -0.50000000000000044 * t11_tmp * -0.86602540378443849 * t13_tmp * t14_tmp
       * t15_tmp) * 2.0, (((((((((((((((((((((((c_q_v1_tmp + k_q_v1_tmp *
    -0.86602540378443849 * t14_tmp) + l_q_v1_tmp * -0.86602540378443849 *
    t14_tmp) - q_v1_tmp_tmp * -0.50000000000000044 * t7_tmp * t10_tmp * t13_tmp)
    - b_q_v1_tmp_tmp * -0.50000000000000044 * t7_tmp * t9_tmp * t13_tmp) -
    q_v1_tmp_tmp_tmp * t8_tmp * t11_tmp * t13_tmp) + g_q_v1_tmp_tmp *
    0.25000000000000044) + h_q_v1_tmp_tmp * 0.25000000000000044) -
    d_q_v1_tmp_tmp * 0.25000000000000044) - e_q_v1_tmp_tmp * 0.25000000000000044)
    - m_q_v1_tmp * -0.86602540378443849 * t13_tmp * t15_tmp) + s_q_v1_tmp) +
    i_q_v1_tmp_tmp * -0.86602540378443849 * t13_tmp * t15_tmp) -
    q_v1_tmp_tmp_tmp * t6_tmp * t11_tmp * t14_tmp * t15_tmp) + n_q_v1_tmp *
    -0.86602540378443849 * t14_tmp) + q_q_v1_tmp * t6_tmp * t10_tmp * t11_tmp *
    -0.86602540378443849 * t15_tmp) + r_q_v1_tmp * t6_tmp * t9_tmp * t11_tmp *
    -0.86602540378443849 * t15_tmp) - q_q_v1_tmp * t7_tmp * t10_tmp * t11_tmp *
    -0.86602540378443849 * t15_tmp) - r_q_v1_tmp * t7_tmp * t9_tmp * t11_tmp *
    -0.86602540378443849 * t15_tmp) + t_q_v1_tmp) - j_q_v1_tmp_tmp *
    -0.86602540378443849 * t14_tmp) - k_q_v1_tmp_tmp * 0.25000000000000044) -
    l_q_v1_tmp_tmp * 0.25000000000000044) + q_q_v1_tmp * t8_tmp * t10_tmp *
    t11_tmp * -0.86602540378443849 * t13_tmp * t14_tmp) + r_q_v1_tmp * t8_tmp *
    t9_tmp * t11_tmp * -0.86602540378443849 * t13_tmp * t14_tmp, 0.0);
}

/*
 * File trailer for Inverse_kinematics_v1.c
 *
 * [EOF]
 */
