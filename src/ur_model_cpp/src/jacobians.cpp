#ifndef JACOBIANS_H_
#define JACOBIANS_H_

#include <ur_model/ur_model.h>

using namespace ur;

void URModel::matrix_J1_0(cc::Jacobian &J1_0,
                           const cc::JointPosition &q) const
{

  J1_0(5, 0) = 1;
}

void URModel::matrix_J2_0(cc::Jacobian &J2_0,
                           const cc::JointPosition &q) const
{

  J2_0(0, 0) = -L2 * ((cos(q(1))) * (sin(q(0))));
  J2_0(0, 1) = -L2 * ((cos(q(0))) * (sin(q(1))));
  J2_0(1, 0) = L2 * ((cos(q(1))) * (cos(q(0))));
  J2_0(1, 1) = -L2 * ((sin(q(1))) * (sin(q(0))));
  J2_0(2, 1) = L2 * (cos(q(1)));
  J2_0(3, 1) = sin(q(0));
  J2_0(4, 1) = -(cos(q(0)));
  J2_0(5, 0) = 1;
}

void URModel::matrix_J3_0(cc::Jacobian &J3_0,
                           const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar b2 = cos(q(0)) * cos(q(1)) * sin(q(2)) + cos(q(0)) * cos(q(2)) * sin(q(1));
  cc::Scalar b4 = (cos(q(1)) * sin(q(2)) + cos(q(2)) * sin(q(1))) * s1;
  cc::Scalar b5 = cos(q(1)) * cos(q(2)) - sin(q(1)) * sin(q(2));

  J3_0(0, 0) = (sin(q(1)) * sin(q(2)) - cos(q(1)) * cos(q(2))) * L3 * s1 + (-L2 * cos(q(1))) * s1;
  J3_0(0, 1) = -L3 * b2 - L2 * cos(q(0)) * sin(q(1));
  J3_0(0, 2) = -L3 * b2;
  J3_0(1, 0) = (cos(q(0)) * cos(q(1)) * cos(q(2)) - cos(q(0)) * sin(q(1)) * sin(q(2))) * L3 + L2 * cos(q(0)) * cos(q(1));
  J3_0(1, 1) = -L3 * b4 - L2 * s1 * sin(q(1));
  J3_0(1, 2) = -L3 * b4;
  J3_0(2, 1) = L3 * b5 + L2 * cos(q(1));
  J3_0(2, 2) = L3 * b5;
  J3_0(3, 1) = s1;
  J3_0(3, 2) = s1;
  J3_0(4, 1) = -(cos(q(0)));
  J3_0(4, 2) = -(cos(q(0)));
  J3_0(5, 0) = 1;
}

void URModel::matrix_J4_0(cc::Jacobian &J4_0,
                           const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c3 = cos(q(2));
  cc::Scalar b2 = (cos(q(1)) * sin(q(2)) + c3 * sin(q(1))) * c1;
  cc::Scalar b4 = (cos(q(1)) * sin(q(2)) + c3 * sin(q(1))) * s1;
  cc::Scalar b5 = c3 * cos(q(1)) - sin(q(1)) * sin(q(2));

  J4_0(0, 0) = (sin(q(1)) * sin(q(2)) - c3 * cos(q(1))) * L3 * s1 + (-L2 * cos(q(1))) * s1 + L4 * c1;
  J4_0(0, 1) = -L3 * b2 - L2 * c1 * sin(q(1));
  J4_0(0, 2) = -L3 * b2;
  J4_0(1, 0) = (c3 * cos(q(1)) - sin(q(1)) * sin(q(2))) * L3 * c1 + (L2 * cos(q(1))) * c1 + L4 * s1;
  J4_0(1, 1) = -L3 * b4 - L2 * s1 * sin(q(1));
  J4_0(1, 2) = -L3 * b4;
  J4_0(2, 1) = L3 * b5 + L2 * cos(q(1));
  J4_0(2, 2) = L3 * b5;
  J4_0(3, 1) = s1;
  J4_0(3, 2) = s1;
  J4_0(3, 3) = s1;
  J4_0(4, 1) = -c1;
  J4_0(4, 2) = -c1;
  J4_0(4, 3) = -c1;
  J4_0(5, 0) = 1;
}

void URModel::matrix_J5_0(cc::Jacobian &J5_0,
                           const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s3 = sin(q(2));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c2 = cos(q(1));
  cc::Scalar c3 = cos(q(2));
  cc::Scalar c4 = cos(q(3));
  cc::Scalar b3 = (c2 * s3 + c3 * s2) * c1;
  cc::Scalar b4 = (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4 + c4 * s2 * s3) * c1;
  cc::Scalar b7 = (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4 + c4 * s2 * s3) * s1;
  cc::Scalar b8 = (c2 * s3 + c3 * s2) * s1;
  cc::Scalar b9 = (c3 * s4 + c4 * s3) * c2 + ((-s2 * s3) * s4 + s2 * c3 * c4);
  cc::Scalar b10 = c2 * c3 - s2 * s3;

  J5_0(0, 0) = (-c3 * s4 - c4 * s3) * L5 * c2 * s1 + (s3 * s4 - c3 * c4) * L5 * s1 * s2 + (-L2 - L3 * c3) * c2 * s1 + (L3 * s3) * s1 * s2 + L4 * c1;
  J5_0(0, 1) = -L3 * b3 - L5 * b4 - L2 * c1 * s2;
  J5_0(0, 2) = -L3 * b3 - L5 * b4;
  J5_0(0, 3) = -L5 * b4;
  J5_0(1, 0) = (L2 * c2 + L3 * c2 * c3 - L3 * s2 * s3 - L5 * s2 * s3 * s4 + L5 * c2 * c3 * s4 + L5 * c2 * c4 * s3 + L5 * c3 * c4 * s2) * c1 + L4 * s1;
  J5_0(1, 1) = -L3 * b8 - L5 * b7 - L2 * s1 * s2;
  J5_0(1, 2) = -L3 * b8 - L5 * b7;
  J5_0(1, 3) = -L5 * b7;
  J5_0(2, 1) = L3 * b10 + L5 * b9 + L2 * c2;
  J5_0(2, 2) = L3 * b10 + L5 * b9;
  J5_0(2, 3) = L5 * b9;
  J5_0(3, 1) = s1;
  J5_0(3, 2) = s1;
  J5_0(3, 3) = s1;
  J5_0(3, 4) = (c2 * c3 * s4 + c2 * c4 * s3 + c3 * c4 * s2 - s2 * s3 * s4) * c1;
  J5_0(4, 1) = -c1;
  J5_0(4, 2) = -c1;
  J5_0(4, 3) = -c1;
  J5_0(4, 4) = (c2 * c3 * s4 + c2 * c4 * s3 + c3 * c4 * s2 - s2 * s3 * s4) * s1;
  J5_0(5, 0) = 1;
  J5_0(5, 4) = (s3 * s4 - c3 * c4) * c2 + ((c4 * s2) * s3 + s2 * c3 * s4);
}

void URModel::matrix_J6_0(cc::Jacobian &J6_0,
                           const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s3 = sin(q(2));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar s5 = sin(q(4));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c2 = cos(q(1));
  cc::Scalar c3 = cos(q(2));
  cc::Scalar c4 = cos(q(3));
  cc::Scalar c5 = cos(q(4));
  cc::Scalar c4s5 = c4 * s5;
  cc::Scalar c1s2 = c1 * s2;
  cc::Scalar b4 = (c1s2 * c4s5 + c1 * c2 * s4 * s5) * c3 + ((c1 * c2) * c4s5 * s3 + (-s4 * s5) * c1s2 * s3);
  cc::Scalar b5 = c3 * c1s2 + c1 * c2 * s3;
  cc::Scalar b6 = (c1s2 * s4 - c1 * c2 * c4) * c3 + ((c1 * c2 * s3) * s4 + s3 * c4 * c1s2);
  cc::Scalar b11 = (c2 * c3 * s4 * s5 + c2 * c4 * s3 * s5 + c3 * c4 * s2 * s5 - s2 * s3 * s4 * s5) * s1;
  cc::Scalar b12 = (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4 + c4 * s2 * s3) * s1;
  cc::Scalar b13 = (c2 * s3 + c3 * s2) * s1;
  cc::Scalar b15 = (c3 * s4 + c4 * s3) * c2 + ((-s2 * s3) * s4 + s2 * c3 * c4);
  cc::Scalar b16 = (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4) * s5 + c4s5 * s2 * s3;
  cc::Scalar b17 = c2 * c3 - s2 * s3;

  J6_0(0, 0) = (c3 * c4s5 - s3 * s4 * s5) * L6 * c2 * s1 + (-c4s5 * s3 - c3 * s4 * s5) * L6 * s1 * s2 + (c1 * c5) * L6 + (-L2 - L3 * c3 - L5 * c3 * s4 - L5 * c4 * s3) * c2 * s1 + (L3 * s3 - L5 * c3 * c4 + L5 * s3 * s4) * s1 * s2 + L4 * c1;
  J6_0(0, 1) = L6 * b4 - L3 * b5 - L5 * b6 - L2 * c1s2;
  J6_0(0, 2) = L6 * b4 - L3 * b5 - L5 * b6;
  J6_0(0, 3) = L6 * b4 - L5 * b6;
  J6_0(0, 4) = (c3 * c5 * c1s2 * s4 - s1 * s5 + c4 * c5 * c1s2 * s3 - c1 * c2 * c3 * c4 * c5 + c1 * c2 * c5 * s3 * s4) * L6;
  J6_0(1, 0) = (s3 * s4 * s5 - c3 * c4 * s5) * L6 * c1 * c2 + (c3 * s4 * s5 + c4 * s3 * s5) * L6 * c1s2 + (c5 * s1) * L6 + (L2 + L3 * c3 + L5 * c3 * s4 + L5 * c4 * s3) * c1 * c2 + (L5 * c3 * c4 - L3 * s3 - L5 * s3 * s4) * c1s2 + L4 * s1;
  J6_0(1, 1) = L6 * b11 - L5 * b12 - L3 * b13 - L2 * s1 * s2;
  J6_0(1, 2) = L6 * b11 - L5 * b12 - L3 * b13;
  J6_0(1, 3) = L6 * b11 - L5 * b12;
  J6_0(1, 4) = (s3 * s4 - c3 * c4) * L6 * c2 * c5 * s1 + (c3 * s2 * s4 + c4 * s2 * s3) * L6 * c5 * s1 + (c1 * s5) * L6;
  J6_0(2, 1) = L3 * b17 + L5 * b15 + L6 * b16 + L2 * c2;
  J6_0(2, 2) = L3 * b17 + L5 * b15 + L6 * b16;
  J6_0(2, 3) = L5 * b15 + L6 * b16;
  J6_0(2, 4) = (c5 * s2 * s3 * s4 - c2 * c3 * c5 * s4 - c2 * c4 * c5 * s3 - c3 * c4 * c5 * s2) * L6;
  J6_0(3, 1) = s1;
  J6_0(3, 2) = s1;
  J6_0(3, 3) = s1;
  J6_0(3, 4) = (c4 * c1s2 + c1 * c2 * s4) * c3 + ((c1 * c2) * c4 * s3 + (-s4) * c1s2 * s3);
  J6_0(3, 5) = (c3 * c1s2 * s4 + c4 * c1s2 * s3 + c1 * c2 * s3 * s4 - c1 * c2 * c3 * c4) * s5 + c5 * s1;
  J6_0(4, 1) = -c1;
  J6_0(4, 2) = -c1;
  J6_0(4, 3) = -c1;
  J6_0(4, 4) = (c2 * c3 * s4 + c2 * c4 * s3 + c3 * c4 * s2 - s2 * s3 * s4) * s1;
  J6_0(4, 5) = (c4s5 * s2 * s3 - c2 * c3 * c4s5 + c2 * s3 * s4 * s5 + c3 * s2 * s4 * s5) * s1 - c1 * c5;
  J6_0(5, 0) = 1;
  J6_0(5, 4) = (s3 * s4 - c3 * c4) * c2 + ((c4 * s2) * s3 + s2 * c3 * s4);
  J6_0(5, 5) = (s2 * s3 * s4 - c2 * c4 * s3 - c3 * c4 * s2 - c2 * c3 * s4) * s5;
}

void URModel::matrix_Jcm1_0(cc::Jacobian &Jcm1_0,
                             const cc::JointPosition &q) const
{

  Jcm1_0(5, 0) = 1;
}

void URModel::matrix_Jcm2_0(cc::Jacobian &Jcm2_0,
                             const cc::JointPosition &q) const
{

  Jcm2_0(0, 0) = -L8 * ((cos(q(1))) * (sin(q(0))));
  Jcm2_0(0, 1) = -L8 * ((cos(q(0))) * (sin(q(1))));
  Jcm2_0(1, 0) = L8 * ((cos(q(1))) * (cos(q(0))));
  Jcm2_0(1, 1) = -L8 * ((sin(q(1))) * (sin(q(0))));
  Jcm2_0(2, 1) = L8 * (cos(q(1)));
  Jcm2_0(3, 1) = sin(q(0));
  Jcm2_0(4, 1) = -(cos(q(0)));
  Jcm2_0(5, 0) = 1;
}

void URModel::matrix_Jcm3_0(cc::Jacobian &Jcm3_0,
                             const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar b2 = cos(q(0)) * cos(q(1)) * sin(q(2)) + cos(q(0)) * cos(q(2)) * sin(q(1));
  cc::Scalar b4 = (cos(q(1)) * sin(q(2)) + cos(q(2)) * sin(q(1))) * s1;
  cc::Scalar b5 = cos(q(1)) * cos(q(2)) - sin(q(1)) * sin(q(2));

  Jcm3_0(0, 0) = (sin(q(1)) * sin(q(2)) - cos(q(1)) * cos(q(2))) * L9 * s1 + (-L2 * cos(q(1))) * s1;
  Jcm3_0(0, 1) = -L9 * b2 - L2 * cos(q(0)) * sin(q(1));
  Jcm3_0(0, 2) = -L9 * b2;
  Jcm3_0(1, 0) = (cos(q(0)) * cos(q(1)) * cos(q(2)) - cos(q(0)) * sin(q(1)) * sin(q(2))) * L9 + L2 * cos(q(0)) * cos(q(1));
  Jcm3_0(1, 1) = -L9 * b4 - L2 * s1 * sin(q(1));
  Jcm3_0(1, 2) = -L9 * b4;
  Jcm3_0(2, 1) = L9 * b5 + L2 * cos(q(1));
  Jcm3_0(2, 2) = L9 * b5;
  Jcm3_0(3, 1) = s1;
  Jcm3_0(3, 2) = s1;
  Jcm3_0(4, 1) = -(cos(q(0)));
  Jcm3_0(4, 2) = -(cos(q(0)));
  Jcm3_0(5, 0) = 1;
}

void URModel::matrix_Jcm4_0(cc::Jacobian &Jcm4_0,
                             const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c3 = cos(q(2));
  cc::Scalar b2 = (cos(q(1)) * sin(q(2)) + c3 * sin(q(1))) * c1;
  cc::Scalar b4 = (cos(q(1)) * sin(q(2)) + c3 * sin(q(1))) * s1;
  cc::Scalar b5 = c3 * cos(q(1)) - sin(q(1)) * sin(q(2));

  Jcm4_0(0, 0) = (sin(q(1)) * sin(q(2)) - c3 * cos(q(1))) * L3 * s1 + (-L2 * cos(q(1))) * s1 + L10 * c1;
  Jcm4_0(0, 1) = -L3 * b2 - L2 * c1 * sin(q(1));
  Jcm4_0(0, 2) = -L3 * b2;
  Jcm4_0(1, 0) = (c3 * cos(q(1)) - sin(q(1)) * sin(q(2))) * L3 * c1 + (L2 * cos(q(1))) * c1 + L10 * s1;
  Jcm4_0(1, 1) = -L3 * b4 - L2 * s1 * sin(q(1));
  Jcm4_0(1, 2) = -L3 * b4;
  Jcm4_0(2, 1) = L3 * b5 + L2 * cos(q(1));
  Jcm4_0(2, 2) = L3 * b5;
  Jcm4_0(3, 1) = s1;
  Jcm4_0(3, 2) = s1;
  Jcm4_0(3, 3) = s1;
  Jcm4_0(4, 1) = -c1;
  Jcm4_0(4, 2) = -c1;
  Jcm4_0(4, 3) = -c1;
  Jcm4_0(5, 0) = 1;
}

void URModel::matrix_Jcm5_0(cc::Jacobian &Jcm5_0,
                             const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s3 = sin(q(2));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c2 = cos(q(1));
  cc::Scalar c3 = cos(q(2));
  cc::Scalar c4 = cos(q(3));
  cc::Scalar b3 = (c2 * s3 + c3 * s2) * c1;
  cc::Scalar b4 = (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4 + c4 * s2 * s3) * c1;
  cc::Scalar b7 = (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4 + c4 * s2 * s3) * s1;
  cc::Scalar b8 = (c2 * s3 + c3 * s2) * s1;
  cc::Scalar b9 = (c3 * s4 + c4 * s3) * c2 + ((-s2 * s3) * s4 + s2 * c3 * c4);
  cc::Scalar b10 = c2 * c3 - s2 * s3;

  Jcm5_0(0, 0) = (-c3 * s4 - c4 * s3) * L11 * c2 * s1 + (s3 * s4 - c3 * c4) * L11 * s1 * s2 + (-L2 - L3 * c3) * c2 * s1 + (L3 * s3) * s1 * s2 + L4 * c1;
  Jcm5_0(0, 1) = -L3 * b3 - L11 * b4 - L2 * c1 * s2;
  Jcm5_0(0, 2) = -L3 * b3 - L11 * b4;
  Jcm5_0(0, 3) = -L11 * b4;
  Jcm5_0(1, 0) = (L2 * c2 + L3 * c2 * c3 - L3 * s2 * s3 - L11 * s2 * s3 * s4 + L11 * c2 * c3 * s4 + L11 * c2 * c4 * s3 + L11 * c3 * c4 * s2) * c1 + L4 * s1;
  Jcm5_0(1, 1) = -L3 * b8 - L11 * b7 - L2 * s1 * s2;
  Jcm5_0(1, 2) = -L3 * b8 - L11 * b7;
  Jcm5_0(1, 3) = -L11 * b7;
  Jcm5_0(2, 1) = L3 * b10 + L11 * b9 + L2 * c2;
  Jcm5_0(2, 2) = L3 * b10 + L11 * b9;
  Jcm5_0(2, 3) = L11 * b9;
  Jcm5_0(3, 1) = s1;
  Jcm5_0(3, 2) = s1;
  Jcm5_0(3, 3) = s1;
  Jcm5_0(3, 4) = (c2 * c3 * s4 + c2 * c4 * s3 + c3 * c4 * s2 - s2 * s3 * s4) * c1;
  Jcm5_0(4, 1) = -c1;
  Jcm5_0(4, 2) = -c1;
  Jcm5_0(4, 3) = -c1;
  Jcm5_0(4, 4) = (c2 * c3 * s4 + c2 * c4 * s3 + c3 * c4 * s2 - s2 * s3 * s4) * s1;
  Jcm5_0(5, 0) = 1;
  Jcm5_0(5, 4) = (s3 * s4 - c3 * c4) * c2 + ((c4 * s2) * s3 + s2 * c3 * s4);
}

void URModel::matrix_Jcm6_0(cc::Jacobian &Jcm6_0,
                             const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s3 = sin(q(2));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar s5 = sin(q(4));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c2 = cos(q(1));
  cc::Scalar c3 = cos(q(2));
  cc::Scalar c4 = cos(q(3));
  cc::Scalar c5 = cos(q(4));
  cc::Scalar c4s5 = c4 * s5;
  cc::Scalar c1s2 = c1 * s2;
  cc::Scalar b4 = (c1s2 * c4s5 + c1 * c2 * s4 * s5) * c3 + ((c1 * c2) * c4s5 * s3 + (-s4 * s5) * c1s2 * s3);
  cc::Scalar b5 = c3 * c1s2 + c1 * c2 * s3;
  cc::Scalar b6 = (c1s2 * s4 - c1 * c2 * c4) * c3 + ((c1 * c2 * s3) * s4 + s3 * c4 * c1s2);
  cc::Scalar b11 = (c2 * c3 * s4 * s5 + c2 * c4 * s3 * s5 + c3 * c4 * s2 * s5 - s2 * s3 * s4 * s5) * s1;
  cc::Scalar b12 = (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4 + c4 * s2 * s3) * s1;
  cc::Scalar b13 = (c2 * s3 + c3 * s2) * s1;
  cc::Scalar b15 = (c3 * s4 + c4 * s3) * c2 + ((-s2 * s3) * s4 + s2 * c3 * c4);
  cc::Scalar b16 = (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4) * s5 + c4s5 * s2 * s3;
  cc::Scalar b17 = c2 * c3 - s2 * s3;

  Jcm6_0(0, 0) = (c3 * c4s5 - s3 * s4 * s5) * L12 * c2 * s1 + (-c4s5 * s3 - c3 * s4 * s5) * L12 * s1 * s2 + (c1 * c5) * L12 + (-L2 - L3 * c3 - L5 * c3 * s4 - L5 * c4 * s3) * c2 * s1 + (L3 * s3 - L5 * c3 * c4 + L5 * s3 * s4) * s1 * s2 + L4 * c1;
  Jcm6_0(0, 1) = L12 * b4 - L5 * b6 - L3 * b5 - L2 * c1s2;
  Jcm6_0(0, 2) = L12 * b4 - L5 * b6 - L3 * b5;
  Jcm6_0(0, 3) = L12 * b4 - L5 * b6;
  Jcm6_0(0, 4) = (c3 * c5 * c1s2 * s4 - s1 * s5 + c4 * c5 * c1s2 * s3 - c1 * c2 * c3 * c4 * c5 + c1 * c2 * c5 * s3 * s4) * L12;
  Jcm6_0(1, 0) = (s3 * s4 * s5 - c3 * c4 * s5) * L12 * c1 * c2 + (c3 * s4 * s5 + c4 * s3 * s5) * L12 * c1s2 + (c5 * s1) * L12 + (L2 + L3 * c3 + L5 * c3 * s4 + L5 * c4 * s3) * c1 * c2 + (L5 * c3 * c4 - L3 * s3 - L5 * s3 * s4) * c1s2 + L4 * s1;
  Jcm6_0(1, 1) = L12 * b11 - L5 * b12 - L3 * b13 - L2 * s1 * s2;
  Jcm6_0(1, 2) = L12 * b11 - L5 * b12 - L3 * b13;
  Jcm6_0(1, 3) = L12 * b11 - L5 * b12;
  Jcm6_0(1, 4) = (s3 * s4 - c3 * c4) * L12 * c2 * c5 * s1 + (c3 * s2 * s4 + c4 * s2 * s3) * L12 * c5 * s1 + (c1 * s5) * L12;
  Jcm6_0(2, 1) = L3 * b17 + L5 * b15 + L12 * b16 + L2 * c2;
  Jcm6_0(2, 2) = L3 * b17 + L5 * b15 + L12 * b16;
  Jcm6_0(2, 3) = L5 * b15 + L12 * b16;
  Jcm6_0(2, 4) = (c5 * s2 * s3 * s4 - c2 * c3 * c5 * s4 - c2 * c4 * c5 * s3 - c3 * c4 * c5 * s2) * L12;
  Jcm6_0(3, 1) = s1;
  Jcm6_0(3, 2) = s1;
  Jcm6_0(3, 3) = s1;
  Jcm6_0(3, 4) = (c4 * c1s2 + c1 * c2 * s4) * c3 + ((c1 * c2) * c4 * s3 + (-s4) * c1s2 * s3);
  Jcm6_0(3, 5) = (c3 * c1s2 * s4 + c4 * c1s2 * s3 + c1 * c2 * s3 * s4 - c1 * c2 * c3 * c4) * s5 + c5 * s1;
  Jcm6_0(4, 1) = -c1;
  Jcm6_0(4, 2) = -c1;
  Jcm6_0(4, 3) = -c1;
  Jcm6_0(4, 4) = (c2 * c3 * s4 + c2 * c4 * s3 + c3 * c4 * s2 - s2 * s3 * s4) * s1;
  Jcm6_0(4, 5) = (c4s5 * s2 * s3 - c2 * c3 * c4s5 + c2 * s3 * s4 * s5 + c3 * s2 * s4 * s5) * s1 - c1 * c5;
  Jcm6_0(5, 0) = 1;
  Jcm6_0(5, 4) = (s3 * s4 - c3 * c4) * c2 + ((c4 * s2) * s3 + s2 * c3 * s4);
  Jcm6_0(5, 5) = (s2 * s3 * s4 - c2 * c4 * s3 - c3 * c4 * s2 - c2 * c3 * s4) * s5;
}

#endif