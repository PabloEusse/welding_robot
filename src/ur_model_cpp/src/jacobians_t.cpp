#ifndef JACOBIANS_T_H_
#define JACOBIANS_T_H_

#include <ur_model/ur_model.h>

using namespace ur;

void URModel::matrix_Jt1_0(cc::Jacobian &Jt1_0,
                            const cc::LinearPosition &t,
                            const cc::JointPosition &q) const
{

  Jt1_0(0, 0) += t(2) * cos(q(0)) - t(0) * sin(q(0));
  Jt1_0(1, 0) += t(0) * cos(q(0)) + t(2) * sin(q(0));
}

void URModel::matrix_Jt2_0(cc::Jacobian &Jt2_0,
                            const cc::LinearPosition &t,
                            const cc::JointPosition &q) const
{

  Jt2_0(0, 0) += t(2) * cos(q(0)) - t(0) * cos(q(1)) * sin(q(0)) + t(1) * sin(q(0)) * sin(q(1));
  Jt2_0(0, 1) += -t(1) * cos(q(0)) * cos(q(1)) - t(0) * cos(q(0)) * sin(q(1));
  Jt2_0(1, 0) += t(2) * sin(q(0)) + t(0) * cos(q(0)) * cos(q(1)) - t(1) * cos(q(0)) * sin(q(1));
  Jt2_0(1, 1) += -t(1) * cos(q(1)) * sin(q(0)) - t(0) * sin(q(0)) * sin(q(1));
  Jt2_0(2, 1) += t(0) * cos(q(1)) - t(1) * sin(q(1));
}

void URModel::matrix_Jt3_0(cc::Jacobian &Jt3_0,
                            const cc::LinearPosition &t,
                            const cc::JointPosition &q) const
{
  cc::Scalar s3 = sin(q(2));
  cc::Scalar b1 = cos(q(1)) * cos(q(2)) * sin(q(0)) - s3 * sin(q(0)) * sin(q(1));
  cc::Scalar b2 = s3 * cos(q(1)) * sin(q(0)) + cos(q(2)) * sin(q(0)) * sin(q(1));
  cc::Scalar b3 = cos(q(0)) * cos(q(1)) * cos(q(2)) - s3 * cos(q(0)) * sin(q(1));
  cc::Scalar b4 = s3 * cos(q(0)) * cos(q(1)) + cos(q(0)) * cos(q(2)) * sin(q(1));
  cc::Scalar b5 = cos(q(1)) * cos(q(2)) - s3 * sin(q(1));
  cc::Scalar b6 = cos(q(2)) * sin(q(1)) + s3 * cos(q(1));

  Jt3_0(0, 0) += b2 * t(1) - b1 * t(0) + t(2) * cos(q(0));
  Jt3_0(0, 1) += -b3 * t(1) - b4 * t(0);
  Jt3_0(0, 2) += -b3 * t(1) - b4 * t(0);
  Jt3_0(1, 0) += b3 * t(0) - b4 * t(1) + t(2) * sin(q(0));
  Jt3_0(1, 1) += -b1 * t(1) - b2 * t(0);
  Jt3_0(1, 2) += -b1 * t(1) - b2 * t(0);
  Jt3_0(2, 1) += b5 * t(0) - b6 * t(1);
  Jt3_0(2, 2) += b5 * t(0) - b6 * t(1);
}

void URModel::matrix_Jt4_0(cc::Jacobian &Jt4_0,
                            const cc::LinearPosition &t,
                            const cc::JointPosition &q) const
{
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar b1 = (s4 * cos(q(2)) * sin(q(0)) + cos(q(3)) * sin(q(0)) * sin(q(2))) * s2 + ((cos(q(1)) * sin(q(0)) * sin(q(2))) * s4 - cos(q(1)) * cos(q(2)) * cos(q(3)) * sin(q(0)));
  cc::Scalar b2 = (cos(q(2)) * cos(q(3)) * sin(q(0)) - s4 * sin(q(0)) * sin(q(2))) * s2 + ((cos(q(1)) * cos(q(2)) * sin(q(0))) * s4 + cos(q(1)) * cos(q(3)) * sin(q(0)) * sin(q(2)));
  cc::Scalar b3 = (s4 * cos(q(0)) * cos(q(2)) + cos(q(0)) * cos(q(3)) * sin(q(2))) * s2 + ((cos(q(0)) * cos(q(1)) * sin(q(2))) * s4 - cos(q(0)) * cos(q(1)) * cos(q(2)) * cos(q(3)));
  cc::Scalar b4 = (cos(q(0)) * cos(q(2)) * cos(q(3)) - s4 * cos(q(0)) * sin(q(2))) * s2 + ((cos(q(0)) * cos(q(1)) * cos(q(2))) * s4 + cos(q(0)) * cos(q(1)) * cos(q(3)) * sin(q(2)));
  cc::Scalar b5 = (cos(q(2)) * cos(q(3)) - s4 * sin(q(2))) * s2 + ((cos(q(1)) * cos(q(2))) * s4 + cos(q(1)) * cos(q(3)) * sin(q(2)));
  cc::Scalar b6 = (cos(q(3)) * sin(q(2)) + s4 * cos(q(2))) * s2 + ((cos(q(1)) * sin(q(2))) * s4 - cos(q(1)) * cos(q(2)) * cos(q(3)));

  Jt4_0(0, 0) += b1 * t(0) - b2 * t(2) + t(1) * cos(q(0));
  Jt4_0(0, 1) += -b4 * t(0) - b3 * t(2);
  Jt4_0(0, 2) += -b4 * t(0) - b3 * t(2);
  Jt4_0(0, 3) += -b4 * t(0) - b3 * t(2);
  Jt4_0(1, 0) += b4 * t(2) - b3 * t(0) + t(1) * sin(q(0));
  Jt4_0(1, 1) += -b2 * t(0) - b1 * t(2);
  Jt4_0(1, 2) += -b2 * t(0) - b1 * t(2);
  Jt4_0(1, 3) += -b2 * t(0) - b1 * t(2);
  Jt4_0(2, 1) += b5 * t(2) - b6 * t(0);
  Jt4_0(2, 2) += b5 * t(2) - b6 * t(0);
  Jt4_0(2, 3) += b5 * t(2) - b6 * t(0);
}

void URModel::matrix_Jt5_0(cc::Jacobian &Jt5_0,
                            const cc::LinearPosition &t,
                            const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s3 = sin(q(2));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar s5 = sin(q(4));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c5 = cos(q(4));
  cc::Scalar b2 = (s1 * s2 * s3 * cos(q(3)) - s1 * cos(q(1)) * cos(q(2)) * cos(q(3)) + s1 * s2 * s4 * cos(q(2)) + s1 * s3 * s4 * cos(q(1))) * c5 + c1 * s5;
  cc::Scalar b3 = (s2 * s3 * s5 * cos(q(3)) - s5 * cos(q(1)) * cos(q(2)) * cos(q(3)) + s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1))) * s1 - c1 * c5;
  cc::Scalar b4 = (s2 * s5 * cos(q(2)) * cos(q(3)) - s2 * s3 * s4 * s5 + s3 * s5 * cos(q(1)) * cos(q(3)) + s4 * s5 * cos(q(1)) * cos(q(2))) * c1;
  cc::Scalar b5 = (c5 * s2 * cos(q(2)) * cos(q(3)) - c5 * s2 * s3 * s4 + c5 * s3 * cos(q(1)) * cos(q(3)) + c5 * s4 * cos(q(1)) * cos(q(2))) * c1;
  cc::Scalar b6 = (s2 * s3 * cos(q(3)) + s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * c1;
  cc::Scalar b7 = (c5 * s2 * s3 * cos(q(3)) - c5 * cos(q(1)) * cos(q(2)) * cos(q(3)) + c5 * s2 * s4 * cos(q(2)) + c5 * s3 * s4 * cos(q(1))) * c1 - s1 * s5;
  cc::Scalar b8 = (s2 * s3 * s5 * cos(q(3)) - s5 * cos(q(1)) * cos(q(2)) * cos(q(3)) + s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1))) * c1 + c5 * s1;
  cc::Scalar b10 = (s2 * s3 * cos(q(3)) + s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * s1;
  cc::Scalar b11 = (s1 * s2 * cos(q(2)) * cos(q(3)) - s1 * s2 * s3 * s4 + s1 * s3 * cos(q(1)) * cos(q(3)) + s1 * s4 * cos(q(1)) * cos(q(2))) * c5;
  cc::Scalar b12 = (s2 * s5 * cos(q(2)) * cos(q(3)) - s2 * s3 * s4 * s5 + s3 * s5 * cos(q(1)) * cos(q(3)) + s4 * s5 * cos(q(1)) * cos(q(2))) * s1;
  cc::Scalar b13 = (s2 * s3 * cos(q(3)) + s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * s5;
  cc::Scalar b14 = (cos(q(2)) * cos(q(3)) - s3 * s4) * s2 + ((cos(q(1)) * cos(q(2))) * s4 + (cos(q(1)) * cos(q(3))) * s3);
  cc::Scalar b15 = (s2 * s3 * cos(q(3)) + s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * c5;

  Jt5_0(0, 0) += (s2 * t(1) * cos(q(2)) * cos(q(3)) - s2 * s3 * s4 * t(1) + s3 * t(1) * cos(q(1)) * cos(q(3)) + s4 * t(1) * cos(q(1)) * cos(q(2))) * s1 + (b2 * t(0) - b3 * t(2));
  Jt5_0(0, 1) += b4 * t(2) - b5 * t(0) + b6 * t(1);
  Jt5_0(0, 2) += b4 * t(2) - b5 * t(0) + b6 * t(1);
  Jt5_0(0, 3) += b4 * t(2) - b5 * t(0) + b6 * t(1);
  Jt5_0(0, 4) += b8 * t(0) + b7 * t(2);
  Jt5_0(1, 0) += (s2 * s3 * s4 * t(1) - s2 * t(1) * cos(q(2)) * cos(q(3)) - s3 * t(1) * cos(q(1)) * cos(q(3)) - s4 * t(1) * cos(q(1)) * cos(q(2))) * c1 + (b8 * t(2) - b7 * t(0));
  Jt5_0(1, 1) += b10 * t(1) - b11 * t(0) + b12 * t(2);
  Jt5_0(1, 2) += b10 * t(1) - b11 * t(0) + b12 * t(2);
  Jt5_0(1, 3) += b10 * t(1) - b11 * t(0) + b12 * t(2);
  Jt5_0(1, 4) += b3 * t(0) + b2 * t(2);
  Jt5_0(2, 1) += b13 * t(2) - b14 * t(1) - b15 * t(0);
  Jt5_0(2, 2) += b13 * t(2) - b14 * t(1) - b15 * t(0);
  Jt5_0(2, 3) += b13 * t(2) - b14 * t(1) - b15 * t(0);
  Jt5_0(2, 4) += (s2 * s3 * s4 * t(2) - s2 * t(2) * cos(q(2)) * cos(q(3)) - s3 * t(2) * cos(q(1)) * cos(q(3)) - s4 * t(2) * cos(q(1)) * cos(q(2))) * c5 + ((-s5 * t(0) * cos(q(1)) * cos(q(2))) * s4 + (-s5 * t(0) * cos(q(1)) * cos(q(3))) * s3 + (-s5 * t(0) * cos(q(2)) * cos(q(3))) * s2 + (s5 * t(0)) * s2 * s3 * s4);
}

void URModel::matrix_Jt6_0(cc::Jacobian &Jt6_0,
                            const cc::LinearPosition &t,
                            const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s3 = sin(q(2));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar s5 = sin(q(4));
  cc::Scalar s6 = sin(q(5));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c2 = cos(q(1));
  cc::Scalar c4 = cos(q(3));
  cc::Scalar c5 = cos(q(4));
  cc::Scalar c6 = cos(q(5));
  cc::Scalar s4s3 = s4 * s3;
  cc::Scalar b1 = (c5 * s4 * cos(q(2)) + c4 * c5 * s3) * c6 * s1 * s2 + (c2 * c5 * s3 * s4 - c2 * c4 * c5 * cos(q(2))) * c6 * s1 + (c1 * s5) * c6 + (c4 * cos(q(2)) - s4s3) * s1 * s2 * s6 + (c2 * s4 * cos(q(2)) + c2 * c4 * s3) * s1 * s6;
  cc::Scalar b2 = (c6 * s2 * s3 * s4 - c2 * c6 * s4 * cos(q(2)) - c4 * c6 * s2 * cos(q(2)) - c2 * c4 * c6 * s3 + c2 * c5 * s3 * s4 * s6 + c4 * c5 * s2 * s3 * s6 - c2 * c4 * c5 * s6 * cos(q(2)) + c5 * s2 * s4 * s6 * cos(q(2))) * s1 + (c1 * s5) * s6;
  cc::Scalar b4 = (c2 * c6 * s4s3 + c4 * c6 * s2 * s3 - c2 * c4 * c6 * cos(q(2)) + c6 * s2 * s4 * cos(q(2)) + c2 * c4 * c5 * s3 * s6 - c5 * s2 * s3 * s4 * s6 + c2 * c5 * s4 * s6 * cos(q(2)) + c4 * c5 * s2 * s6 * cos(q(2))) * c1;
  cc::Scalar b5 = (c2 * c4 * s3 * s5 - s2 * s3 * s4 * s5 + c2 * s4 * s5 * cos(q(2)) + c4 * s2 * s5 * cos(q(2))) * c1;
  cc::Scalar b6 = (c2 * c4 * s6 * cos(q(2)) - c2 * s3 * s4 * s6 - c4 * s2 * s3 * s6 - c5 * c6 * s2 * s4s3 - s2 * s4 * s6 * cos(q(2)) + c2 * c4 * c5 * c6 * s3 + c2 * c5 * c6 * s4 * cos(q(2)) + c4 * c5 * c6 * s2 * cos(q(2))) * c1;
  cc::Scalar b10 = (s4s3 - c4 * cos(q(2))) * c1 * c2 * c5 * c6 + (s4 * s6 * cos(q(2)) + c4 * s3 * s6) * c1 * c2 + (s2 * s4 * cos(q(2)) + c4 * s2 * s3) * c1 * c5 * c6 + (c4 * s2 * s6 * cos(q(2)) - s2 * s3 * s4 * s6) * c1 + (-s1 * s5) * c6;
  cc::Scalar b11 = (c4 * cos(q(2)) - s4s3) * c1 * c6 * s2 + (c2 * s4 * cos(q(2)) + c2 * c4 * s3) * c1 * c6 + (-c5 * s4 * cos(q(2)) - c4 * c5 * s3) * c1 * s2 * s6 + (c2 * c4 * c5 * cos(q(2)) - c2 * c5 * s4s3) * c1 * s6 + (s1 * s5) * s6;
  cc::Scalar b13 = (c2 * c6 * s4s3 + c4 * c6 * s2 * s3 - c5 * s2 * s6 * s4s3 - c2 * c4 * c6 * cos(q(2)) + c6 * s2 * s4 * cos(q(2)) + c2 * c4 * c5 * s3 * s6 + c2 * c5 * s4 * s6 * cos(q(2)) + c4 * c5 * s2 * s6 * cos(q(2))) * s1;
  cc::Scalar b14 = (c2 * c4 * s3 * s5 - s2 * s3 * s4 * s5 + c2 * s4 * s5 * cos(q(2)) + c4 * s2 * s5 * cos(q(2))) * s1;
  cc::Scalar b15 = (c2 * s6 * s4s3 + c4 * s2 * s3 * s6 - c2 * c4 * s6 * cos(q(2)) + s2 * s4 * s6 * cos(q(2)) - c2 * c4 * c5 * c6 * s3 + c5 * c6 * s2 * s3 * s4 - c2 * c5 * c6 * s4 * cos(q(2)) - c4 * c5 * c6 * s2 * cos(q(2))) * s1;
  cc::Scalar b19 = (s2 * s4 * cos(q(2)) + c2 * s3 * s4 + c4 * s2 * s3 - c2 * c4 * cos(q(2))) * s5;
  cc::Scalar b20 = (s4 * s6 * cos(q(2)) + c4 * s3 * s6 + c5 * c6 * s3 * s4 - c4 * c5 * c6 * cos(q(2))) * c2 + ((c5 * c6 * s2) * c4 * s3 + (s2 * cos(q(2))) * c4 * s6 + (-s2 * s4) * s3 * s6 + c5 * c6 * s2 * s4 * cos(q(2)));
  cc::Scalar b21 = (c6 * s4 * cos(q(2)) + c4 * c6 * s3 - c5 * s3 * s4 * s6 + c4 * c5 * s6 * cos(q(2))) * c2 + ((c4 * cos(q(2))) * c6 * s2 + (-c4 * c5 * s3 * s6) * s2 + (-c5 * s6 * cos(q(2))) * s2 * s4 + (-s3) * c6 * s2 * s4);

  Jt6_0(0, 0) += (c4 * cos(q(2)) - s4s3) * c2 * s1 * s5 * t(2) + (-s2 * s4 * cos(q(2)) - c4 * s2 * s3) * s1 * s5 * t(2) + (c1 * c5) * t(2) + (b1 * t(0) - b2 * t(1));
  Jt6_0(0, 1) += b4 * t(1) - b6 * t(0) + b5 * t(2);
  Jt6_0(0, 2) += b4 * t(1) - b6 * t(0) + b5 * t(2);
  Jt6_0(0, 3) += b4 * t(1) - b6 * t(0) + b5 * t(2);
  Jt6_0(0, 4) += (s4s3 * t(2) - c4 * t(2) * cos(q(2))) * c1 * c2 * c5 + (c6 * s4s3 * t(0) - s3 * s4 * s6 * t(1) - c4 * c6 * t(0) * cos(q(2)) + c4 * s6 * t(1) * cos(q(2))) * c1 * c2 * s5 + (c4 * s2 * s3 * t(2) + s2 * s4 * t(2) * cos(q(2))) * c1 * c5 + (c4 * c6 * s2 * s3 * t(0) - s2 * s4 * s6 * t(1) * cos(q(2)) - c4 * s2 * s3 * s6 * t(1) + c6 * s2 * s4 * t(0) * cos(q(2))) * c1 * s5 + (c6 * s1 * t(0) - s1 * s6 * t(1)) * c5 + (-s1 * t(2)) * s5;
  Jt6_0(0, 5) += b10 * t(1) - b11 * t(0);
  Jt6_0(1, 0) += (s4s3 - c4 * cos(q(2))) * c1 * c2 * s5 * t(2) + (s2 * s4 * cos(q(2)) + c4 * s2 * s3) * c1 * s5 * t(2) + (c5 * s1) * t(2) - (b10 * t(0) + b11 * t(1));
  Jt6_0(1, 1) += b13 * t(1) + b15 * t(0) + b14 * t(2);
  Jt6_0(1, 2) += b13 * t(1) + b15 * t(0) + b14 * t(2);
  Jt6_0(1, 3) += b13 * t(1) + b15 * t(0) + b14 * t(2);
  Jt6_0(1, 4) += (s4s3 * t(2) - c4 * t(2) * cos(q(2))) * c2 * c5 * s1 + (c6 * s4s3 * t(0) - s6 * s4s3 * t(1) - c4 * c6 * t(0) * cos(q(2)) + c4 * s6 * t(1) * cos(q(2))) * c2 * s1 * s5 + (c4 * s2 * s3 * t(2) + s2 * s4 * t(2) * cos(q(2))) * c5 * s1 + (c1 * s6 * t(1) - c1 * c6 * t(0)) * c5 + (c4 * c6 * s2 * s3 * t(0) - s2 * s4 * s6 * t(1) * cos(q(2)) - c4 * s2 * s3 * s6 * t(1) + c6 * s2 * s4 * t(0) * cos(q(2))) * s1 * s5 + (c1 * t(2)) * s5;
  Jt6_0(1, 5) += b1 * t(1) + b2 * t(0);
  Jt6_0(2, 1) += b19 * t(2) - b20 * t(0) - b21 * t(1);
  Jt6_0(2, 2) += b19 * t(2) - b20 * t(0) - b21 * t(1);
  Jt6_0(2, 3) += b19 * t(2) - b20 * t(0) - b21 * t(1);
  Jt6_0(2, 4) += (c6 * t(0) - s6 * t(1)) * s2 * s3 * s4 * s5 + (c5 * t(2)) * s2 * s3 * s4 + (c4 * s6 * t(1) * cos(q(2)) - c4 * c6 * t(0) * cos(q(2))) * s2 * s5 + (-c4 * c5 * t(2) * cos(q(2))) * s2 + (c2 * c4 * s6 * t(1) - c2 * c4 * c6 * t(0)) * s3 * s5 + (-c2 * c4 * c5 * t(2)) * s3 + (c2 * s6 * t(1) * cos(q(2)) - c2 * c6 * t(0) * cos(q(2))) * s4 * s5 + (-c2 * c5 * t(2) * cos(q(2))) * s4;
  Jt6_0(2, 5) += (-c5 * s4 * cos(q(2)) - c4 * c5 * s3) * c2 * c6 * t(1) + (c4 * t(0) * cos(q(2)) - s4s3 * t(0)) * c2 * c6 + (s4s3 - c4 * cos(q(2))) * c2 * s6 * t(1) + (-c4 * c5 * s3 * t(0) - c5 * s4 * t(0) * cos(q(2))) * c2 * s6 + (c5 * s2 * s3 * s4 - c4 * c5 * s2 * cos(q(2))) * c6 * t(1) + (-c4 * s2 * s3 * t(0) - s2 * s4 * t(0) * cos(q(2))) * c6 + (s2 * s4 * cos(q(2)) + c4 * s2 * s3) * s6 * t(1) + (c5 * s2 * s3 * s4 * t(0) - c4 * c5 * s2 * t(0) * cos(q(2))) * s6;
}

void URModel::matrix_Jtcm1_0(cc::Jacobian &Jtcm1_0,
                              const cc::LinearPosition &t,
                              const cc::JointPosition &q) const
{

  Jtcm1_0(0, 0) += -t(1) * cos(q(0)) - t(0) * sin(q(0));
  Jtcm1_0(1, 0) += t(0) * cos(q(0)) - t(1) * sin(q(0));
}

void URModel::matrix_Jtcm2_0(cc::Jacobian &Jtcm2_0,
                              const cc::LinearPosition &t,
                              const cc::JointPosition &q) const
{

  Jtcm2_0(0, 0) += t(2) * cos(q(0)) - t(0) * cos(q(1)) * sin(q(0)) + t(1) * sin(q(0)) * sin(q(1));
  Jtcm2_0(0, 1) += -t(1) * cos(q(0)) * cos(q(1)) - t(0) * cos(q(0)) * sin(q(1));
  Jtcm2_0(1, 0) += t(2) * sin(q(0)) + t(0) * cos(q(0)) * cos(q(1)) - t(1) * cos(q(0)) * sin(q(1));
  Jtcm2_0(1, 1) += -t(1) * cos(q(1)) * sin(q(0)) - t(0) * sin(q(0)) * sin(q(1));
  Jtcm2_0(2, 1) += t(0) * cos(q(1)) - t(1) * sin(q(1));
}

void URModel::matrix_Jtcm3_0(cc::Jacobian &Jtcm3_0,
                              const cc::LinearPosition &t,
                              const cc::JointPosition &q) const
{
  cc::Scalar s3 = sin(q(2));
  cc::Scalar b1 = cos(q(1)) * cos(q(2)) * sin(q(0)) - s3 * sin(q(0)) * sin(q(1));
  cc::Scalar b2 = s3 * cos(q(1)) * sin(q(0)) + cos(q(2)) * sin(q(0)) * sin(q(1));
  cc::Scalar b3 = cos(q(0)) * cos(q(1)) * cos(q(2)) - s3 * cos(q(0)) * sin(q(1));
  cc::Scalar b4 = s3 * cos(q(0)) * cos(q(1)) + cos(q(0)) * cos(q(2)) * sin(q(1));
  cc::Scalar b5 = cos(q(1)) * cos(q(2)) - s3 * sin(q(1));
  cc::Scalar b6 = cos(q(2)) * sin(q(1)) + s3 * cos(q(1));

  Jtcm3_0(0, 0) += b2 * t(1) - b1 * t(0) + t(2) * cos(q(0));
  Jtcm3_0(0, 1) += -b3 * t(1) - b4 * t(0);
  Jtcm3_0(0, 2) += -b3 * t(1) - b4 * t(0);
  Jtcm3_0(1, 0) += b3 * t(0) - b4 * t(1) + t(2) * sin(q(0));
  Jtcm3_0(1, 1) += -b1 * t(1) - b2 * t(0);
  Jtcm3_0(1, 2) += -b1 * t(1) - b2 * t(0);
  Jtcm3_0(2, 1) += b5 * t(0) - b6 * t(1);
  Jtcm3_0(2, 2) += b5 * t(0) - b6 * t(1);
}

void URModel::matrix_Jtcm4_0(cc::Jacobian &Jtcm4_0,
                              const cc::LinearPosition &t,
                              const cc::JointPosition &q) const
{
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar c2 = cos(q(1));
  cc::Scalar b1 = (s4 * cos(q(2)) * sin(q(0)) + cos(q(3)) * sin(q(0)) * sin(q(2))) * s2 + ((sin(q(0)) * sin(q(2))) * c2 * s4 + (-cos(q(2)) * cos(q(3)) * sin(q(0))) * c2);
  cc::Scalar b2 = (s4 * cos(q(2)) * sin(q(0)) + cos(q(3)) * sin(q(0)) * sin(q(2))) * c2 + ((cos(q(2)) * cos(q(3)) * sin(q(0))) * s2 + (-sin(q(0)) * sin(q(2))) * s2 * s4);
  cc::Scalar b3 = (s4 * cos(q(0)) * cos(q(2)) + cos(q(0)) * cos(q(3)) * sin(q(2))) * s2 + ((cos(q(0)) * sin(q(2))) * c2 * s4 + (-cos(q(0)) * cos(q(2)) * cos(q(3))) * c2);
  cc::Scalar b4 = (s4 * cos(q(0)) * cos(q(2)) + cos(q(0)) * cos(q(3)) * sin(q(2))) * c2 + ((cos(q(0)) * cos(q(2)) * cos(q(3))) * s2 + (-cos(q(0)) * sin(q(2))) * s2 * s4);
  cc::Scalar b5 = (s4 * sin(q(2)) - cos(q(2)) * cos(q(3))) * c2 + ((cos(q(3)) * sin(q(2))) * s2 + cos(q(2)) * s2 * s4);
  cc::Scalar b6 = (cos(q(3)) * sin(q(2)) + s4 * cos(q(2))) * c2 + ((cos(q(2)) * cos(q(3))) * s2 + (-sin(q(2))) * s2 * s4);

  Jtcm4_0(0, 0) += b1 * t(0) + b2 * t(1) + t(2) * cos(q(0));
  Jtcm4_0(0, 1) += b3 * t(1) - b4 * t(0);
  Jtcm4_0(0, 2) += b3 * t(1) - b4 * t(0);
  Jtcm4_0(0, 3) += b3 * t(1) - b4 * t(0);
  Jtcm4_0(1, 0) += t(2) * sin(q(0)) - b4 * t(1) - b3 * t(0);
  Jtcm4_0(1, 1) += b1 * t(1) - b2 * t(0);
  Jtcm4_0(1, 2) += b1 * t(1) - b2 * t(0);
  Jtcm4_0(1, 3) += b1 * t(1) - b2 * t(0);
  Jtcm4_0(2, 1) += -b5 * t(0) - b6 * t(1);
  Jtcm4_0(2, 2) += -b5 * t(0) - b6 * t(1);
  Jtcm4_0(2, 3) += -b5 * t(0) - b6 * t(1);
}

void URModel::matrix_Jtcm5_0(cc::Jacobian &Jtcm5_0,
                              const cc::LinearPosition &t,
                              const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s3 = sin(q(2));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar s5 = sin(q(4));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c5 = cos(q(4));
  cc::Scalar s5s3 = s5 * s3;
  cc::Scalar b1 = (s1 * s2 * s3 * cos(q(3)) - s1 * cos(q(1)) * cos(q(2)) * cos(q(3)) + s1 * s2 * s4 * cos(q(2)) + s1 * s3 * s4 * cos(q(1))) * c5 + c1 * s5;
  cc::Scalar b3 = (s2 * s5s3 * cos(q(3)) + s4 * s5s3 * cos(q(1)) - s5 * cos(q(1)) * cos(q(2)) * cos(q(3)) + s2 * s4 * s5 * cos(q(2))) * s1 - c1 * c5;
  cc::Scalar b4 = (s2 * s5 * cos(q(2)) * cos(q(3)) - s2 * s4 * s5s3 + s3 * s5 * cos(q(1)) * cos(q(3)) + s4 * s5 * cos(q(1)) * cos(q(2))) * c1;
  cc::Scalar b5 = (c5 * s2 * cos(q(2)) * cos(q(3)) - c5 * s2 * s3 * s4 + c5 * s3 * cos(q(1)) * cos(q(3)) + c5 * s4 * cos(q(1)) * cos(q(2))) * c1;
  cc::Scalar b6 = (s2 * s3 * cos(q(3)) + s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * c1;
  cc::Scalar b7 = (c5 * s2 * s3 * cos(q(3)) - c5 * cos(q(1)) * cos(q(2)) * cos(q(3)) + c5 * s2 * s4 * cos(q(2)) + c5 * s3 * s4 * cos(q(1))) * c1 - s1 * s5;
  cc::Scalar b8 = (s2 * s3 * s5 * cos(q(3)) - s5 * cos(q(1)) * cos(q(2)) * cos(q(3)) + s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1))) * c1 + c5 * s1;
  cc::Scalar b10 = (s2 * s5 * cos(q(2)) * cos(q(3)) - s2 * s3 * s4 * s5 + s3 * s5 * cos(q(1)) * cos(q(3)) + s4 * s5 * cos(q(1)) * cos(q(2))) * s1;
  cc::Scalar b11 = (s1 * s2 * cos(q(2)) * cos(q(3)) - s1 * s2 * s3 * s4 + s1 * s3 * cos(q(1)) * cos(q(3)) + s1 * s4 * cos(q(1)) * cos(q(2))) * c5;
  cc::Scalar b12 = (s2 * s3 * cos(q(3)) + s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * s1;
  cc::Scalar b13 = (cos(q(2)) * cos(q(3)) - s3 * s4) * s2 + ((cos(q(1)) * cos(q(2))) * s4 + (cos(q(1)) * cos(q(3))) * s3);
  cc::Scalar b14 = (s2 * s3 * cos(q(3)) + s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - cos(q(1)) * cos(q(2)) * cos(q(3))) * c5;
  cc::Scalar b15 = (s5s3 * cos(q(3)) + s4 * s5 * cos(q(2))) * s2 + (cos(q(1)) * s4 * s5s3 + (-cos(q(1)) * cos(q(2)) * cos(q(3))) * s5);

  Jtcm5_0(0, 0) += (s2 * s3 * s4 * t(2) - s2 * t(2) * cos(q(2)) * cos(q(3)) - s3 * t(2) * cos(q(1)) * cos(q(3)) - s4 * t(2) * cos(q(1)) * cos(q(2))) * s1 + (b1 * t(0) - b3 * t(1));
  Jtcm5_0(0, 1) += b4 * t(1) - b5 * t(0) - b6 * t(2);
  Jtcm5_0(0, 2) += b4 * t(1) - b5 * t(0) - b6 * t(2);
  Jtcm5_0(0, 3) += b4 * t(1) - b5 * t(0) - b6 * t(2);
  Jtcm5_0(0, 4) += b7 * t(1) + b8 * t(0);
  Jtcm5_0(1, 0) += (s2 * t(2) * cos(q(2)) * cos(q(3)) - s2 * s3 * s4 * t(2) + s3 * t(2) * cos(q(1)) * cos(q(3)) + s4 * t(2) * cos(q(1)) * cos(q(2))) * c1 + (b8 * t(1) - b7 * t(0));
  Jtcm5_0(1, 1) += b10 * t(1) - b11 * t(0) - b12 * t(2);
  Jtcm5_0(1, 2) += b10 * t(1) - b11 * t(0) - b12 * t(2);
  Jtcm5_0(1, 3) += b10 * t(1) - b11 * t(0) - b12 * t(2);
  Jtcm5_0(1, 4) += b1 * t(1) + b3 * t(0);
  Jtcm5_0(2, 1) += b13 * t(2) - b14 * t(0) + b15 * t(1);
  Jtcm5_0(2, 2) += b13 * t(2) - b14 * t(0) + b15 * t(1);
  Jtcm5_0(2, 3) += b13 * t(2) - b14 * t(0) + b15 * t(1);
  Jtcm5_0(2, 4) += (s2 * s3 * s4 * t(1) - s2 * t(1) * cos(q(2)) * cos(q(3)) - s3 * t(1) * cos(q(1)) * cos(q(3)) - s4 * t(1) * cos(q(1)) * cos(q(2))) * c5 + ((-s5 * t(0) * cos(q(1)) * cos(q(2))) * s4 + (-s5 * t(0) * cos(q(1)) * cos(q(3))) * s3 + (-s5 * t(0) * cos(q(2)) * cos(q(3))) * s2 + (s5 * t(0)) * s2 * s3 * s4);
}

void URModel::matrix_Jtcm6_0(cc::Jacobian &Jtcm6_0,
                              const cc::LinearPosition &t,
                              const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s3 = sin(q(2));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar s5 = sin(q(4));
  cc::Scalar s6 = sin(q(5));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c2 = cos(q(1));
  cc::Scalar c4 = cos(q(3));
  cc::Scalar c5 = cos(q(4));
  cc::Scalar c6 = cos(q(5));
  cc::Scalar s4s3 = s4 * s3;
  cc::Scalar b1 = (c5 * s4 * cos(q(2)) + c4 * c5 * s3) * c6 * s1 * s2 + (c2 * c5 * s3 * s4 - c2 * c4 * c5 * cos(q(2))) * c6 * s1 + (c1 * s5) * c6 + (c4 * cos(q(2)) - s4s3) * s1 * s2 * s6 + (c2 * s4 * cos(q(2)) + c2 * c4 * s3) * s1 * s6;
  cc::Scalar b2 = (c6 * s2 * s3 * s4 - c2 * c6 * s4 * cos(q(2)) - c4 * c6 * s2 * cos(q(2)) - c2 * c4 * c6 * s3 + c2 * c5 * s3 * s4 * s6 + c4 * c5 * s2 * s3 * s6 - c2 * c4 * c5 * s6 * cos(q(2)) + c5 * s2 * s4 * s6 * cos(q(2))) * s1 + (c1 * s5) * s6;
  cc::Scalar b4 = (c2 * c6 * s4s3 + c4 * c6 * s2 * s3 - c2 * c4 * c6 * cos(q(2)) + c6 * s2 * s4 * cos(q(2)) + c2 * c4 * c5 * s3 * s6 - c5 * s2 * s3 * s4 * s6 + c2 * c5 * s4 * s6 * cos(q(2)) + c4 * c5 * s2 * s6 * cos(q(2))) * c1;
  cc::Scalar b5 = (c2 * c4 * s3 * s5 - s2 * s3 * s4 * s5 + c2 * s4 * s5 * cos(q(2)) + c4 * s2 * s5 * cos(q(2))) * c1;
  cc::Scalar b6 = (c2 * c4 * s6 * cos(q(2)) - c2 * s3 * s4 * s6 - c4 * s2 * s3 * s6 - c5 * c6 * s2 * s4s3 - s2 * s4 * s6 * cos(q(2)) + c2 * c4 * c5 * c6 * s3 + c2 * c5 * c6 * s4 * cos(q(2)) + c4 * c5 * c6 * s2 * cos(q(2))) * c1;
  cc::Scalar b10 = (s4s3 - c4 * cos(q(2))) * c1 * c2 * c5 * c6 + (s4 * s6 * cos(q(2)) + c4 * s3 * s6) * c1 * c2 + (s2 * s4 * cos(q(2)) + c4 * s2 * s3) * c1 * c5 * c6 + (c4 * s2 * s6 * cos(q(2)) - s2 * s3 * s4 * s6) * c1 + (-s1 * s5) * c6;
  cc::Scalar b11 = (c4 * cos(q(2)) - s4s3) * c1 * c6 * s2 + (c2 * s4 * cos(q(2)) + c2 * c4 * s3) * c1 * c6 + (-c5 * s4 * cos(q(2)) - c4 * c5 * s3) * c1 * s2 * s6 + (c2 * c4 * c5 * cos(q(2)) - c2 * c5 * s4s3) * c1 * s6 + (s1 * s5) * s6;
  cc::Scalar b13 = (c2 * c6 * s4s3 + c4 * c6 * s2 * s3 - c5 * s2 * s6 * s4s3 - c2 * c4 * c6 * cos(q(2)) + c6 * s2 * s4 * cos(q(2)) + c2 * c4 * c5 * s3 * s6 + c2 * c5 * s4 * s6 * cos(q(2)) + c4 * c5 * s2 * s6 * cos(q(2))) * s1;
  cc::Scalar b14 = (c2 * c4 * s3 * s5 - s2 * s3 * s4 * s5 + c2 * s4 * s5 * cos(q(2)) + c4 * s2 * s5 * cos(q(2))) * s1;
  cc::Scalar b15 = (c2 * s6 * s4s3 + c4 * s2 * s3 * s6 - c2 * c4 * s6 * cos(q(2)) + s2 * s4 * s6 * cos(q(2)) - c2 * c4 * c5 * c6 * s3 + c5 * c6 * s2 * s3 * s4 - c2 * c5 * c6 * s4 * cos(q(2)) - c4 * c5 * c6 * s2 * cos(q(2))) * s1;
  cc::Scalar b19 = (s2 * s4 * cos(q(2)) + c2 * s3 * s4 + c4 * s2 * s3 - c2 * c4 * cos(q(2))) * s5;
  cc::Scalar b20 = (s4 * s6 * cos(q(2)) + c4 * s3 * s6 + c5 * c6 * s3 * s4 - c4 * c5 * c6 * cos(q(2))) * c2 + ((c5 * c6 * s2) * c4 * s3 + (s2 * cos(q(2))) * c4 * s6 + (-s2 * s4) * s3 * s6 + c5 * c6 * s2 * s4 * cos(q(2)));
  cc::Scalar b21 = (c6 * s4 * cos(q(2)) + c4 * c6 * s3 - c5 * s3 * s4 * s6 + c4 * c5 * s6 * cos(q(2))) * c2 + ((c4 * cos(q(2))) * c6 * s2 + (-c4 * c5 * s3 * s6) * s2 + (-c5 * s6 * cos(q(2))) * s2 * s4 + (-s3) * c6 * s2 * s4);

  Jtcm6_0(0, 0) += (c4 * cos(q(2)) - s4s3) * c2 * s1 * s5 * t(2) + (-s2 * s4 * cos(q(2)) - c4 * s2 * s3) * s1 * s5 * t(2) + (c1 * c5) * t(2) + (b1 * t(0) - b2 * t(1));
  Jtcm6_0(0, 1) += b4 * t(1) - b6 * t(0) + b5 * t(2);
  Jtcm6_0(0, 2) += b4 * t(1) - b6 * t(0) + b5 * t(2);
  Jtcm6_0(0, 3) += b4 * t(1) - b6 * t(0) + b5 * t(2);
  Jtcm6_0(0, 4) += (s4s3 * t(2) - c4 * t(2) * cos(q(2))) * c1 * c2 * c5 + (c6 * s4s3 * t(0) - s3 * s4 * s6 * t(1) - c4 * c6 * t(0) * cos(q(2)) + c4 * s6 * t(1) * cos(q(2))) * c1 * c2 * s5 + (c4 * s2 * s3 * t(2) + s2 * s4 * t(2) * cos(q(2))) * c1 * c5 + (c4 * c6 * s2 * s3 * t(0) - s2 * s4 * s6 * t(1) * cos(q(2)) - c4 * s2 * s3 * s6 * t(1) + c6 * s2 * s4 * t(0) * cos(q(2))) * c1 * s5 + (c6 * s1 * t(0) - s1 * s6 * t(1)) * c5 + (-s1 * t(2)) * s5;
  Jtcm6_0(0, 5) += b10 * t(1) - b11 * t(0);
  Jtcm6_0(1, 0) += (s4s3 - c4 * cos(q(2))) * c1 * c2 * s5 * t(2) + (s2 * s4 * cos(q(2)) + c4 * s2 * s3) * c1 * s5 * t(2) + (c5 * s1) * t(2) - (b10 * t(0) + b11 * t(1));
  Jtcm6_0(1, 1) += b13 * t(1) + b15 * t(0) + b14 * t(2);
  Jtcm6_0(1, 2) += b13 * t(1) + b15 * t(0) + b14 * t(2);
  Jtcm6_0(1, 3) += b13 * t(1) + b15 * t(0) + b14 * t(2);
  Jtcm6_0(1, 4) += (s4s3 * t(2) - c4 * t(2) * cos(q(2))) * c2 * c5 * s1 + (c6 * s4s3 * t(0) - s6 * s4s3 * t(1) - c4 * c6 * t(0) * cos(q(2)) + c4 * s6 * t(1) * cos(q(2))) * c2 * s1 * s5 + (c4 * s2 * s3 * t(2) + s2 * s4 * t(2) * cos(q(2))) * c5 * s1 + (c1 * s6 * t(1) - c1 * c6 * t(0)) * c5 + (c4 * c6 * s2 * s3 * t(0) - s2 * s4 * s6 * t(1) * cos(q(2)) - c4 * s2 * s3 * s6 * t(1) + c6 * s2 * s4 * t(0) * cos(q(2))) * s1 * s5 + (c1 * t(2)) * s5;
  Jtcm6_0(1, 5) += b1 * t(1) + b2 * t(0);
  Jtcm6_0(2, 1) += b19 * t(2) - b20 * t(0) - b21 * t(1);
  Jtcm6_0(2, 2) += b19 * t(2) - b20 * t(0) - b21 * t(1);
  Jtcm6_0(2, 3) += b19 * t(2) - b20 * t(0) - b21 * t(1);
  Jtcm6_0(2, 4) += (c6 * t(0) - s6 * t(1)) * s2 * s3 * s4 * s5 + (c5 * t(2)) * s2 * s3 * s4 + (c4 * s6 * t(1) * cos(q(2)) - c4 * c6 * t(0) * cos(q(2))) * s2 * s5 + (-c4 * c5 * t(2) * cos(q(2))) * s2 + (c2 * c4 * s6 * t(1) - c2 * c4 * c6 * t(0)) * s3 * s5 + (-c2 * c4 * c5 * t(2)) * s3 + (c2 * s6 * t(1) * cos(q(2)) - c2 * c6 * t(0) * cos(q(2))) * s4 * s5 + (-c2 * c5 * t(2) * cos(q(2))) * s4;
  Jtcm6_0(2, 5) += (-c5 * s4 * cos(q(2)) - c4 * c5 * s3) * c2 * c6 * t(1) + (c4 * t(0) * cos(q(2)) - s4s3 * t(0)) * c2 * c6 + (s4s3 - c4 * cos(q(2))) * c2 * s6 * t(1) + (-c4 * c5 * s3 * t(0) - c5 * s4 * t(0) * cos(q(2))) * c2 * s6 + (c5 * s2 * s3 * s4 - c4 * c5 * s2 * cos(q(2))) * c6 * t(1) + (-c4 * s2 * s3 * t(0) - s2 * s4 * t(0) * cos(q(2))) * c6 + (s2 * s4 * cos(q(2)) + c4 * s2 * s3) * s6 * t(1) + (c5 * s2 * s3 * s4 * t(0) - c4 * c5 * s2 * t(0) * cos(q(2))) * s6;
}

#endif