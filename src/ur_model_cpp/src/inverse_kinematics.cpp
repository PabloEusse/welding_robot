#ifndef INVERSE_TRANSFORMATIONS_H_
#define INVERSE_TRANSFORMATIONS_H_

#include <ur_model/ur_model.h>

using namespace ur;

template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

void URModel::matrix_IK(IKSolutions &Qs, const cc::HomogeneousTransformation &T, cc::Scalar q6_d) const
{
  static const cc::Scalar ZERO_THRESHOLD = 1e-8;
  unsigned char n = 0;

  // joint q1
  cc::Scalar q1[2];
  cc::Scalar t60_x = T(0, 3) - T(0, 2) * L6;
  cc::Scalar t60_y = T(1, 3) - T(1, 2) * L6;
  cc::Scalar r = t60_x * t60_x + t60_y * t60_y;

  cc::Scalar arccos = std::acos(L4 / std::sqrt(r));
  cc::Scalar arctan = std::atan2(t60_y, t60_x);

  cc::Scalar pos = arctan + arccos;
  cc::Scalar neg = arctan - arccos;
  if (std::fabs(pos) < ZERO_THRESHOLD)
    pos = cc::Scalar(0.0);
  if (std::fabs(neg) < ZERO_THRESHOLD)
    neg = cc::Scalar(0.0);
  q1[0] = pos + cc::Scalar(M_PI_2);
  q1[1] = neg + cc::Scalar(M_PI_2);

  // joint q5
  cc::Scalar q5[2][2];
  for (size_t i = 0; i < 2; ++i)
  {
    cc::Scalar numer = T(0, 3) * std::sin(q1[i]) - T(1, 3) * std::cos(q1[i]) - L4;
    arccos = std::acos(numer / L6);
    q5[i][0] = +arccos;
    q5[i][1] = -arccos;
  }

  for (size_t i = 0; i < 2; ++i)
  {
    cc::Scalar c1 = std::cos(q1[i]);
    cc::Scalar s1 = std::sin(q1[i]);
    for (size_t j = 0; j < 2; ++j)
    {
      cc::Scalar c5 = std::cos(q5[i][j]);
      cc::Scalar s5 = std::sin(q5[i][j]);

      // joint 6
      cc::Scalar q6;
      if (std::fabs(s5) < ZERO_THRESHOLD)
        q6 = q6_d;
      else
      {
        q6 = std::atan2(sgn<float>(s5) * (-T(0, 1) * s1 + T(1, 1) * c1), sgn<float>(s5) * (T(0, 0) * s1 - T(1, 0) * c1));
        if (q6 < ZERO_THRESHOLD)
          q6 = cc::Scalar(0.0);
      }

      // remaining rrr planar robot
      cc::Scalar q2[2];
      cc::Scalar q3[2];
      cc::Scalar q4[2];
      cc::Scalar c6 = std::cos(q6);
      cc::Scalar s6 = std::sin(q6);

      cc::Scalar x04x = -s5 * (T(0, 2) * c1 + T(1, 2) * s1) - c5 * (s6 * (T(0, 1) * c1 + T(1, 1) * s1) - c6 * (T(0, 0) * c1 + T(1, 0) * s1));
      cc::Scalar x04y = c5 * (T(2, 0) * c6 - T(2, 1) * s6) - T(2, 2) * s5;
      cc::Scalar p41_x = L5 * (c6 * (T(0, 1) * c1 + T(1, 1) * s1) + s6 * (T(0, 0) * c1 + T(1, 0) * s1)) - L6 * (T(0, 2) * c1 + T(1, 2) * s1) + T(0, 3) * c1 + T(1, 3) * s1;
      cc::Scalar p41_y = T(2, 3) - L1 - L6 * T(2, 2) + L5 * (T(2, 1) * c6 + T(2, 0) * s6);
      cc::Scalar c3 = (p41_x * p41_x + p41_y * p41_y - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
      arccos = std::acos(c3);
      q3[0] = +arccos;
      q3[1] = -arccos;
      cc::Scalar denom = L2 * L2 + L3 * L3 + 2 * L2 * L3 * L3;
      cc::Scalar s3 = std::sin(arccos);
      cc::Scalar a = L2 + L2 * c3;
      cc::Scalar b = L3 * s3;
      q2[0] = std::atan2((a * p41_y - b * p41_x) / denom, (a * p41_x + b * p41_y) / denom);
      q2[1] = std::atan2((a * p41_y + b * p41_x) / denom, (a * p41_x - b * p41_y) / denom);

      cc::Scalar c43_0 = cos(q2[0] + q3[0]);
      cc::Scalar s43_0 = sin(q2[0] + q3[0]);
      cc::Scalar c43_1 = cos(q2[1] + q3[1]);
      cc::Scalar s43_1 = sin(q2[1] + q3[1]);

      q4[0] = std::atan2(c43_0 * x04y - s43_0 * x04x, x04x * c43_0 + x04y * s43_0);
      q4[1] = std::atan2(c43_1 * x04y - s43_1 * x04x, x04x * c43_1 + x04y * s43_1);

      for (size_t k = 0; k < 2; ++k)
      {
        if (std::fabs(q2[k]) < ZERO_THRESHOLD)
          q2[k] = cc::Scalar(0.0);
        if (std::fabs(q4[k]) < ZERO_THRESHOLD)
          q4[k] = cc::Scalar(0.0);

        Qs(0, n) = q1[i];
        Qs(1, n) = q2[k];
        Qs(2, n) = q3[k];
        Qs(3, n) = q4[k];
        Qs(4, n) = q5[i][j];
        Qs(5, n) = q6;
        n++;
      }
    }
  }
}

#endif