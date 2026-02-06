#ifndef JACOBIANS_P_H_
#define JACOBIANS_P_H_

#include <ur_model/ur_model.h>

using namespace ur;

void URModel::matrix_J1_0p(cc::Jacobian &J1_0p,
                            const cc::JointPosition &q,
                            const cc::JointVelocity &qP) const
{
}

void URModel::matrix_J2_0p(cc::Jacobian &J2_0p,
                            const cc::JointPosition &q,
                            const cc::JointVelocity &qP) const
{
  cc::Scalar a37 = L2 * qP(1);
  cc::Scalar a38 = L2 * qP(0);

  J2_0p(0, 0) = a37 * sin(q(0)) * sin(q(1)) - a38 * cos(q(0)) * cos(q(1));
  J2_0p(0, 1) = a38 * sin(q(0)) * sin(q(1)) - a37 * cos(q(0)) * cos(q(1));
  J2_0p(1, 0) = -a37 * cos(q(0)) * sin(q(1)) - a38 * cos(q(1)) * sin(q(0));
  J2_0p(1, 1) = -a37 * cos(q(1)) * sin(q(0)) - a38 * cos(q(0)) * sin(q(1));
  J2_0p(2, 1) = -a37 * (sin(q(1)));
  J2_0p(3, 1) = (cos(q(0))) * qP(0);
  J2_0p(4, 1) = qP(0) * (sin(q(0)));
}

void URModel::matrix_J3_0p(cc::Jacobian &J3_0p,
                            const cc::JointPosition &q,
                            const cc::JointVelocity &qP) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar a37 = L2 * qP(1);
  cc::Scalar a38 = L2 * qP(0);
  cc::Scalar a39 = L3 * qP(0);
  cc::Scalar b1 = cos(q(0)) * cos(q(1)) * cos(q(2)) - s2 * cos(q(0)) * sin(q(2));
  cc::Scalar b2 = (cos(q(1)) * sin(q(2)) + s2 * cos(q(2))) * s1;
  cc::Scalar b3 = (cos(q(1)) * cos(q(2)) - s2 * sin(q(2))) * s1;
  cc::Scalar b4 = s2 * cos(q(0)) * cos(q(2)) + cos(q(0)) * cos(q(1)) * sin(q(2));
  cc::Scalar b5 = cos(q(1)) * sin(q(2)) + s2 * cos(q(2));
  cc::Scalar d1 = (qP(1) + qP(2)) * L3;

  J3_0p(0, 0) = b2 * d1 - a39 * b1 - a38 * cos(q(0)) * cos(q(1)) + a37 * s1 * s2;
  J3_0p(0, 1) = a39 * b2 - b1 * d1 - a37 * cos(q(0)) * cos(q(1)) + a38 * s1 * s2;
  J3_0p(0, 2) = a39 * b2 - b1 * d1;
  J3_0p(1, 0) = -a39 * b3 - b4 * d1 - a37 * s2 * cos(q(0)) - a38 * s1 * cos(q(1));
  J3_0p(1, 1) = -a39 * b4 - b3 * d1 - a37 * s1 * cos(q(1)) - a38 * s2 * cos(q(0));
  J3_0p(1, 2) = -a39 * b4 - b3 * d1;
  J3_0p(2, 1) = -b5 * d1 - a37 * s2;
  J3_0p(2, 2) = -b5 * d1;
  J3_0p(3, 1) = (cos(q(0))) * qP(0);
  J3_0p(3, 2) = (cos(q(0))) * qP(0);
  J3_0p(4, 1) = qP(0) * s1;
  J3_0p(4, 2) = qP(0) * s1;
}

void URModel::matrix_J4_0p(cc::Jacobian &J4_0p,
                            const cc::JointPosition &q,
                            const cc::JointVelocity &qP) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c3 = cos(q(2));
  cc::Scalar a37 = L2 * qP(1);
  cc::Scalar a38 = L2 * qP(0);
  cc::Scalar a39 = L4 * qP(0);
  cc::Scalar a40 = L3 * qP(0);
  cc::Scalar b1 = (c3 * cos(q(1)) - s2 * sin(q(2))) * c1;
  cc::Scalar b2 = (cos(q(1)) * sin(q(2)) + c3 * s2) * s1;
  cc::Scalar b3 = (c3 * cos(q(1)) - s2 * sin(q(2))) * s1;
  cc::Scalar b4 = (cos(q(1)) * sin(q(2)) + c3 * s2) * c1;
  cc::Scalar b5 = cos(q(1)) * sin(q(2)) + c3 * s2;
  cc::Scalar d1 = (qP(1) + qP(2)) * L3;

  J4_0p(0, 0) = (a37 * s2 - a39) * s1 + (b2 * d1 - a40 * b1 - a38 * c1 * cos(q(1)));
  J4_0p(0, 1) = a40 * b2 - b1 * d1 + a38 * s1 * s2 - a37 * c1 * cos(q(1));
  J4_0p(0, 2) = a40 * b2 - b1 * d1;
  J4_0p(1, 0) = (a39 - a37 * s2) * c1 - (a40 * b3 + b4 * d1 + a38 * s1 * cos(q(1)));
  J4_0p(1, 1) = -a40 * b4 - b3 * d1 - a37 * s1 * cos(q(1)) - a38 * c1 * s2;
  J4_0p(1, 2) = -a40 * b4 - b3 * d1;
  J4_0p(2, 1) = -b5 * d1 - a37 * s2;
  J4_0p(2, 2) = -b5 * d1;
  J4_0p(3, 1) = c1 * qP(0);
  J4_0p(3, 2) = c1 * qP(0);
  J4_0p(3, 3) = c1 * qP(0);
  J4_0p(4, 1) = qP(0) * s1;
  J4_0p(4, 2) = qP(0) * s1;
  J4_0p(4, 3) = qP(0) * s1;
}

void URModel::matrix_J5_0p(cc::Jacobian &J5_0p,
                            const cc::JointPosition &q,
                            const cc::JointVelocity &qP) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c4 = cos(q(3));
  cc::Scalar a37 = L2 * qP(1);
  cc::Scalar a38 = L2 * qP(0);
  cc::Scalar a39 = L4 * qP(0);
  cc::Scalar a40 = L3 * qP(0);
  cc::Scalar a43 = L5 * qP(0);
  cc::Scalar b1 = (c4 * s2 * sin(q(2)) + s2 * s4 * cos(q(2)) - c4 * cos(q(1)) * cos(q(2)) + s4 * cos(q(1)) * sin(q(2))) * s1;
  cc::Scalar b2 = (c4 * s2 * cos(q(2)) - s2 * s4 * sin(q(2)) + c4 * cos(q(1)) * sin(q(2)) + s4 * cos(q(1)) * cos(q(2))) * c1;
  cc::Scalar b3 = (cos(q(1)) * cos(q(2)) - s2 * sin(q(2))) * c1;
  cc::Scalar b4 = (cos(q(1)) * sin(q(2)) + s2 * cos(q(2))) * s1;
  cc::Scalar b5 = (c4 * s2 * sin(q(2)) + s2 * s4 * cos(q(2)) - c4 * cos(q(1)) * cos(q(2)) + s4 * cos(q(1)) * sin(q(2))) * c1;
  cc::Scalar b6 = (c4 * s2 * cos(q(2)) - s2 * s4 * sin(q(2)) + c4 * cos(q(1)) * sin(q(2)) + s4 * cos(q(1)) * cos(q(2))) * s1;
  cc::Scalar b7 = (cos(q(1)) * cos(q(2)) - s2 * sin(q(2))) * s1;
  cc::Scalar b8 = (cos(q(1)) * sin(q(2)) + s2 * cos(q(2))) * c1;
  cc::Scalar b9 = (s2 * sin(q(2)) - cos(q(1)) * cos(q(2))) * c4 + ((cos(q(1)) * sin(q(2))) * s4 + cos(q(2)) * s2 * s4);
  cc::Scalar b10 = cos(q(1)) * sin(q(2)) + s2 * cos(q(2));
  cc::Scalar d1 = (qP(1) + qP(2) + qP(3)) * L5;
  cc::Scalar d2 = (qP(1) + qP(2)) * L3;
  cc::Scalar d3 = qP(1) + qP(2) + qP(3);

  J5_0p(0, 0) = (a37 * s2 - a39) * s1 + (b1 * d1 - a43 * b2 - a40 * b3 + b4 * d2 - a38 * c1 * cos(q(1)));
  J5_0p(0, 1) = a40 * b4 + a43 * b1 - b2 * d1 - b3 * d2 + a38 * s1 * s2 - a37 * c1 * cos(q(1));
  J5_0p(0, 2) = a40 * b4 + a43 * b1 - b2 * d1 - b3 * d2;
  J5_0p(0, 3) = a43 * b1 - b2 * d1;
  J5_0p(1, 0) = (a39 - a37 * s2) * c1 - (a40 * b7 + a43 * b6 + b5 * d1 + b8 * d2 + a38 * s1 * cos(q(1)));
  J5_0p(1, 1) = -a40 * b8 - a43 * b5 - b6 * d1 - b7 * d2 - a37 * s1 * cos(q(1)) - a38 * c1 * s2;
  J5_0p(1, 2) = -a40 * b8 - a43 * b5 - b6 * d1 - b7 * d2;
  J5_0p(1, 3) = -a43 * b5 - b6 * d1;
  J5_0p(2, 1) = -b9 * d1 - b10 * d2 - a37 * s2;
  J5_0p(2, 2) = -b9 * d1 - b10 * d2;
  J5_0p(2, 3) = -b9 * d1;
  J5_0p(3, 1) = c1 * qP(0);
  J5_0p(3, 2) = c1 * qP(0);
  J5_0p(3, 3) = c1 * qP(0);
  J5_0p(3, 4) = -b5 * d3 - b6 * qP(0);
  J5_0p(4, 1) = qP(0) * s1;
  J5_0p(4, 2) = qP(0) * s1;
  J5_0p(4, 3) = qP(0) * s1;
  J5_0p(4, 4) = b2 * qP(0) - b1 * d3;
  J5_0p(5, 4) = (c4 * s2 * cos(q(2)) - s2 * s4 * sin(q(2)) + c4 * cos(q(1)) * sin(q(2)) + s4 * cos(q(1)) * cos(q(2))) * d3;
}

void URModel::matrix_J6_0p(cc::Jacobian &J6_0p,
                            const cc::JointPosition &q,
                            const cc::JointVelocity &qP) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s3 = sin(q(2));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar s5 = sin(q(4));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c4 = cos(q(3));
  cc::Scalar c5 = cos(q(4));
  cc::Scalar s2s1 = s2 * s1;
  cc::Scalar c4s5 = c4 * s5;
  cc::Scalar a37 = L2 * qP(1);
  cc::Scalar a38 = L2 * qP(0);
  cc::Scalar a39 = L6 * qP(0);
  cc::Scalar a40 = L6 * qP(4);
  cc::Scalar a41 = L4 * qP(0);
  cc::Scalar a42 = L3 * qP(0);
  cc::Scalar a45 = L5 * qP(0);
  cc::Scalar b1 = (s2s1 * cos(q(2)) + s1 * s3 * cos(q(1))) * s4 + ((-s1 * cos(q(1)) * cos(q(2))) * c4 + c4 * s3 * s2s1);
  cc::Scalar b2 = (c4 * s2 * cos(q(2)) + c4 * s3 * cos(q(1)) + s4 * cos(q(1)) * cos(q(2)) - s2 * s3 * s4) * c1;
  cc::Scalar b3 = (s2s1 * cos(q(2)) + s1 * s3 * cos(q(1))) * c4s5 + ((s1 * s5 * cos(q(1)) * cos(q(2))) * s4 + (-s5) * s3 * s4 * s2s1);
  cc::Scalar b4 = (cos(q(1)) * cos(q(2)) - s2 * s3) * c1;
  cc::Scalar b5 = (c4s5 * s2 * s3 - c4s5 * cos(q(1)) * cos(q(2)) + s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1))) * c1 + c5 * s1;
  cc::Scalar b6 = (s4 * s2s1 * cos(q(2)) + c4 * s3 * s2s1 + s1 * s3 * s4 * cos(q(1)) - c4 * s1 * cos(q(1)) * cos(q(2))) * c5 + c1 * s5;
  cc::Scalar b7 = s2s1 * cos(q(2)) + s1 * s3 * cos(q(1));
  cc::Scalar b8 = (c4 * c5 * s2 * cos(q(2)) - c5 * s2 * s3 * s4 + c4 * c5 * s3 * cos(q(1)) + c5 * s4 * cos(q(1)) * cos(q(2))) * c1;
  cc::Scalar b9 = (c4s5 * s2 * s3 - c4s5 * cos(q(1)) * cos(q(2)) + s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1))) * c1;
  cc::Scalar b10 = (c4 * s2 * s5 * cos(q(2)) - s2 * s3 * s4 * s5 + c4 * s3 * s5 * cos(q(1)) + s4 * s5 * cos(q(1)) * cos(q(2))) * c1;
  cc::Scalar b11 = (s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - c4 * cos(q(1)) * cos(q(2)) + c4 * s2 * s3) * c1;
  cc::Scalar b12 = (c4 * s3 * cos(q(1)) + s4 * cos(q(1)) * cos(q(2)) - s2 * s3 * s4) * s1 + (s2s1 * cos(q(2))) * c4;
  cc::Scalar b13 = (cos(q(1)) * cos(q(2)) - s2 * s3) * s1;
  cc::Scalar b14 = (c4 * c5 * s2 * s3 + c5 * s2 * s4 * cos(q(2)) + c5 * s3 * s4 * cos(q(1)) - c4 * c5 * cos(q(1)) * cos(q(2))) * c1 - s1 * s5;
  cc::Scalar b15 = (s4 * s2s1 * cos(q(2)) + c4 * s3 * s2s1 + s1 * s3 * s4 * cos(q(1)) - c4 * s1 * cos(q(1)) * cos(q(2))) * s5 - c1 * c5;
  cc::Scalar b16 = (s2 * cos(q(2)) + s3 * cos(q(1))) * c1;
  cc::Scalar b17 = (s4 * s2s1 * cos(q(2)) + c4 * s3 * s2s1 + s1 * s3 * s4 * cos(q(1)) - c4 * s1 * cos(q(1)) * cos(q(2))) * s5;
  cc::Scalar b18 = (s3 * cos(q(1))) * c4 * c5 * s1 + (s2s1 * cos(q(2))) * c4 * c5 + (cos(q(1)) * cos(q(2)) - s2 * s3) * c5 * s1 * s4;
  cc::Scalar b19 = (s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - c4 * cos(q(1)) * cos(q(2)) + c4 * s2 * s3) * c5;
  cc::Scalar b20 = (c4 * s3 + s4 * cos(q(2))) * s2 + (cos(q(1)) * s3 * s4 + (-cos(q(1)) * cos(q(2))) * c4);
  cc::Scalar b21 = (s2 * cos(q(2)) + s3 * cos(q(1))) * c4s5 + ((s5 * cos(q(1)) * cos(q(2))) * s4 + (-s5) * s2 * s3 * s4);
  cc::Scalar b22 = s2 * cos(q(2)) + s3 * cos(q(1));
  cc::Scalar d1 = (qP(1) + qP(2) + qP(3)) * L5;
  cc::Scalar d2 = (qP(1) + qP(2) + qP(3)) * L6;
  cc::Scalar d3 = (qP(1) + qP(2)) * L3;
  cc::Scalar d4 = qP(1) + qP(2) + qP(3);

  J6_0p(0, 0) = b1 * d1 - a40 * b6 - a42 * b4 - a45 * b2 - a39 * b5 - b3 * d2 + b7 * d3 - a41 * s1 + a37 * s2s1 - a38 * c1 * cos(q(1));
  J6_0p(0, 1) = a45 * b1 - a39 * b3 + a40 * b8 + a42 * b7 - b2 * d1 - b4 * d3 - b9 * d2 + a38 * s2s1 - a37 * c1 * cos(q(1));
  J6_0p(0, 2) = a45 * b1 - a39 * b3 + a40 * b8 + a42 * b7 - b2 * d1 - b4 * d3 - b9 * d2;
  J6_0p(0, 3) = a45 * b1 - a39 * b3 + a40 * b8 - b2 * d1 - b9 * d2;
  J6_0p(0, 4) = b8 * d2 - a40 * b5 - a39 * b6;
  J6_0p(1, 0) = (a41 - a37 * s2) * c1 + (a40 * b14 - a39 * b15 - a42 * b13 - a45 * b12 + b10 * d2 - b11 * d1 - b16 * d3 - a38 * s1 * cos(q(1)));
  J6_0p(1, 1) = a39 * b10 - a45 * b11 + a40 * b18 - a42 * b16 - b12 * d1 - b13 * d3 - b17 * d2 - a37 * s1 * cos(q(1)) - a38 * c1 * s2;
  J6_0p(1, 2) = a39 * b10 - a45 * b11 + a40 * b18 - a42 * b16 - b12 * d1 - b13 * d3 - b17 * d2;
  J6_0p(1, 3) = a39 * b10 - a45 * b11 + a40 * b18 - b12 * d1 - b17 * d2;
  J6_0p(1, 4) = a39 * b14 - a40 * b15 + b18 * d2;
  J6_0p(2, 1) = a40 * b19 - b20 * d1 + b21 * d2 - b22 * d3 - a37 * s2;
  J6_0p(2, 2) = a40 * b19 - b20 * d1 + b21 * d2 - b22 * d3;
  J6_0p(2, 3) = a40 * b19 - b20 * d1 + b21 * d2;
  J6_0p(2, 4) = a40 * b21 + b19 * d2;
  J6_0p(3, 1) = c1 * qP(0);
  J6_0p(3, 2) = c1 * qP(0);
  J6_0p(3, 3) = c1 * qP(0);
  J6_0p(3, 4) = -b11 * d4 - b12 * qP(0);
  J6_0p(3, 5) = b10 * d4 - b15 * qP(0) + b14 * qP(4);
  J6_0p(4, 1) = qP(0) * s1;
  J6_0p(4, 2) = qP(0) * s1;
  J6_0p(4, 3) = qP(0) * s1;
  J6_0p(4, 4) = b2 * qP(0) - b1 * d4;
  J6_0p(4, 5) = b3 * d4 + b5 * qP(0) + b6 * qP(4);
  J6_0p(5, 4) = (c4 * s2 * cos(q(2)) + c4 * s3 * cos(q(1)) + s4 * cos(q(1)) * cos(q(2)) - s2 * s3 * s4) * d4;
  J6_0p(5, 5) = (d4 * s2 * s3 * s5 - c5 * qP(4) * s2 * cos(q(2)) - c5 * qP(4) * s3 * cos(q(1)) - d4 * s5 * cos(q(1)) * cos(q(2))) * c4 + ((s4 * s5 * cos(q(1))) * d4 * s3 + (s4 * s5 * cos(q(2))) * d4 * s2 + (c5 * qP(4) * s4) * s2 * s3 - c5 * qP(4) * s4 * cos(q(1)) * cos(q(2)));
}

void URModel::matrix_Jcm1_0p(cc::Jacobian &Jcm1_0p,
                              const cc::JointPosition &q,
                              const cc::JointVelocity &qP) const
{
}

void URModel::matrix_Jcm2_0p(cc::Jacobian &Jcm2_0p,
                              const cc::JointPosition &q,
                              const cc::JointVelocity &qP) const
{
  cc::Scalar a37 = L8 * qP(1);
  cc::Scalar a38 = L8 * qP(0);

  Jcm2_0p(0, 0) = a37 * sin(q(0)) * sin(q(1)) - a38 * cos(q(0)) * cos(q(1));
  Jcm2_0p(0, 1) = a38 * sin(q(0)) * sin(q(1)) - a37 * cos(q(0)) * cos(q(1));
  Jcm2_0p(1, 0) = -a37 * cos(q(0)) * sin(q(1)) - a38 * cos(q(1)) * sin(q(0));
  Jcm2_0p(1, 1) = -a37 * cos(q(1)) * sin(q(0)) - a38 * cos(q(0)) * sin(q(1));
  Jcm2_0p(2, 1) = -a37 * (sin(q(1)));
  Jcm2_0p(3, 1) = (cos(q(0))) * qP(0);
  Jcm2_0p(4, 1) = qP(0) * (sin(q(0)));
}

void URModel::matrix_Jcm3_0p(cc::Jacobian &Jcm3_0p,
                              const cc::JointPosition &q,
                              const cc::JointVelocity &qP) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar a37 = L2 * qP(1);
  cc::Scalar a38 = L2 * qP(0);
  cc::Scalar a39 = L9 * qP(0);
  cc::Scalar b1 = cos(q(0)) * cos(q(1)) * cos(q(2)) - s2 * cos(q(0)) * sin(q(2));
  cc::Scalar b2 = (cos(q(1)) * sin(q(2)) + s2 * cos(q(2))) * s1;
  cc::Scalar b3 = (cos(q(1)) * cos(q(2)) - s2 * sin(q(2))) * s1;
  cc::Scalar b4 = s2 * cos(q(0)) * cos(q(2)) + cos(q(0)) * cos(q(1)) * sin(q(2));
  cc::Scalar b5 = cos(q(1)) * sin(q(2)) + s2 * cos(q(2));
  cc::Scalar d1 = (qP(1) + qP(2)) * L9;

  Jcm3_0p(0, 0) = b2 * d1 - a39 * b1 - a38 * cos(q(0)) * cos(q(1)) + a37 * s1 * s2;
  Jcm3_0p(0, 1) = a39 * b2 - b1 * d1 - a37 * cos(q(0)) * cos(q(1)) + a38 * s1 * s2;
  Jcm3_0p(0, 2) = a39 * b2 - b1 * d1;
  Jcm3_0p(1, 0) = -a39 * b3 - b4 * d1 - a37 * s2 * cos(q(0)) - a38 * s1 * cos(q(1));
  Jcm3_0p(1, 1) = -a39 * b4 - b3 * d1 - a37 * s1 * cos(q(1)) - a38 * s2 * cos(q(0));
  Jcm3_0p(1, 2) = -a39 * b4 - b3 * d1;
  Jcm3_0p(2, 1) = -b5 * d1 - a37 * s2;
  Jcm3_0p(2, 2) = -b5 * d1;
  Jcm3_0p(3, 1) = (cos(q(0))) * qP(0);
  Jcm3_0p(3, 2) = (cos(q(0))) * qP(0);
  Jcm3_0p(4, 1) = qP(0) * s1;
  Jcm3_0p(4, 2) = qP(0) * s1;
}

void URModel::matrix_Jcm4_0p(cc::Jacobian &Jcm4_0p,
                              const cc::JointPosition &q,
                              const cc::JointVelocity &qP) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c3 = cos(q(2));
  cc::Scalar a37 = L2 * qP(1);
  cc::Scalar a38 = L2 * qP(0);
  cc::Scalar a39 = L10 * qP(0);
  cc::Scalar a40 = L3 * qP(0);
  cc::Scalar b1 = (c3 * cos(q(1)) - s2 * sin(q(2))) * c1;
  cc::Scalar b2 = (cos(q(1)) * sin(q(2)) + c3 * s2) * s1;
  cc::Scalar b3 = (c3 * cos(q(1)) - s2 * sin(q(2))) * s1;
  cc::Scalar b4 = (cos(q(1)) * sin(q(2)) + c3 * s2) * c1;
  cc::Scalar b5 = cos(q(1)) * sin(q(2)) + c3 * s2;
  cc::Scalar d1 = (qP(1) + qP(2)) * L3;

  Jcm4_0p(0, 0) = (a37 * s2 - a39) * s1 + (b2 * d1 - a40 * b1 - a38 * c1 * cos(q(1)));
  Jcm4_0p(0, 1) = a40 * b2 - b1 * d1 + a38 * s1 * s2 - a37 * c1 * cos(q(1));
  Jcm4_0p(0, 2) = a40 * b2 - b1 * d1;
  Jcm4_0p(1, 0) = (a39 - a37 * s2) * c1 - (a40 * b3 + b4 * d1 + a38 * s1 * cos(q(1)));
  Jcm4_0p(1, 1) = -a40 * b4 - b3 * d1 - a37 * s1 * cos(q(1)) - a38 * c1 * s2;
  Jcm4_0p(1, 2) = -a40 * b4 - b3 * d1;
  Jcm4_0p(2, 1) = -b5 * d1 - a37 * s2;
  Jcm4_0p(2, 2) = -b5 * d1;
  Jcm4_0p(3, 1) = c1 * qP(0);
  Jcm4_0p(3, 2) = c1 * qP(0);
  Jcm4_0p(3, 3) = c1 * qP(0);
  Jcm4_0p(4, 1) = qP(0) * s1;
  Jcm4_0p(4, 2) = qP(0) * s1;
  Jcm4_0p(4, 3) = qP(0) * s1;
}

void URModel::matrix_Jcm5_0p(cc::Jacobian &Jcm5_0p,
                              const cc::JointPosition &q,
                              const cc::JointVelocity &qP) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c4 = cos(q(3));
  cc::Scalar a37 = L2 * qP(1);
  cc::Scalar a38 = L2 * qP(0);
  cc::Scalar a39 = L4 * qP(0);
  cc::Scalar a40 = L3 * qP(0);
  cc::Scalar a43 = L11 * qP(0);
  cc::Scalar b1 = (c4 * s2 * sin(q(2)) + s2 * s4 * cos(q(2)) - c4 * cos(q(1)) * cos(q(2)) + s4 * cos(q(1)) * sin(q(2))) * s1;
  cc::Scalar b2 = (c4 * s2 * cos(q(2)) - s2 * s4 * sin(q(2)) + c4 * cos(q(1)) * sin(q(2)) + s4 * cos(q(1)) * cos(q(2))) * c1;
  cc::Scalar b3 = (cos(q(1)) * cos(q(2)) - s2 * sin(q(2))) * c1;
  cc::Scalar b4 = (cos(q(1)) * sin(q(2)) + s2 * cos(q(2))) * s1;
  cc::Scalar b5 = (c4 * s2 * sin(q(2)) + s2 * s4 * cos(q(2)) - c4 * cos(q(1)) * cos(q(2)) + s4 * cos(q(1)) * sin(q(2))) * c1;
  cc::Scalar b6 = (c4 * s2 * cos(q(2)) - s2 * s4 * sin(q(2)) + c4 * cos(q(1)) * sin(q(2)) + s4 * cos(q(1)) * cos(q(2))) * s1;
  cc::Scalar b7 = (cos(q(1)) * cos(q(2)) - s2 * sin(q(2))) * s1;
  cc::Scalar b8 = (cos(q(1)) * sin(q(2)) + s2 * cos(q(2))) * c1;
  cc::Scalar b9 = (s2 * sin(q(2)) - cos(q(1)) * cos(q(2))) * c4 + ((cos(q(1)) * sin(q(2))) * s4 + cos(q(2)) * s2 * s4);
  cc::Scalar b10 = cos(q(1)) * sin(q(2)) + s2 * cos(q(2));
  cc::Scalar d1 = (qP(1) + qP(2) + qP(3)) * L11;
  cc::Scalar d2 = (qP(1) + qP(2)) * L3;
  cc::Scalar d3 = qP(1) + qP(2) + qP(3);

  Jcm5_0p(0, 0) = (a37 * s2 - a39) * s1 + (b1 * d1 - a43 * b2 - a40 * b3 + b4 * d2 - a38 * c1 * cos(q(1)));
  Jcm5_0p(0, 1) = a40 * b4 + a43 * b1 - b2 * d1 - b3 * d2 + a38 * s1 * s2 - a37 * c1 * cos(q(1));
  Jcm5_0p(0, 2) = a40 * b4 + a43 * b1 - b2 * d1 - b3 * d2;
  Jcm5_0p(0, 3) = a43 * b1 - b2 * d1;
  Jcm5_0p(1, 0) = (a39 - a37 * s2) * c1 - (a40 * b7 + a43 * b6 + b5 * d1 + b8 * d2 + a38 * s1 * cos(q(1)));
  Jcm5_0p(1, 1) = -a40 * b8 - a43 * b5 - b6 * d1 - b7 * d2 - a37 * s1 * cos(q(1)) - a38 * c1 * s2;
  Jcm5_0p(1, 2) = -a40 * b8 - a43 * b5 - b6 * d1 - b7 * d2;
  Jcm5_0p(1, 3) = -a43 * b5 - b6 * d1;
  Jcm5_0p(2, 1) = -b9 * d1 - b10 * d2 - a37 * s2;
  Jcm5_0p(2, 2) = -b9 * d1 - b10 * d2;
  Jcm5_0p(2, 3) = -b9 * d1;
  Jcm5_0p(3, 1) = c1 * qP(0);
  Jcm5_0p(3, 2) = c1 * qP(0);
  Jcm5_0p(3, 3) = c1 * qP(0);
  Jcm5_0p(3, 4) = -b5 * d3 - b6 * qP(0);
  Jcm5_0p(4, 1) = qP(0) * s1;
  Jcm5_0p(4, 2) = qP(0) * s1;
  Jcm5_0p(4, 3) = qP(0) * s1;
  Jcm5_0p(4, 4) = b2 * qP(0) - b1 * d3;
  Jcm5_0p(5, 4) = (c4 * s2 * cos(q(2)) - s2 * s4 * sin(q(2)) + c4 * cos(q(1)) * sin(q(2)) + s4 * cos(q(1)) * cos(q(2))) * d3;
}

void URModel::matrix_Jcm6_0p(cc::Jacobian &Jcm6_0p,
                              const cc::JointPosition &q,
                              const cc::JointVelocity &qP) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s3 = sin(q(2));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar s5 = sin(q(4));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c4 = cos(q(3));
  cc::Scalar c5 = cos(q(4));
  cc::Scalar s2s1 = s2 * s1;
  cc::Scalar c4s5 = c4 * s5;
  cc::Scalar a37 = L2 * qP(1);
  cc::Scalar a38 = L2 * qP(0);
  cc::Scalar a39 = L12 * qP(0);
  cc::Scalar a40 = L12 * qP(4);
  cc::Scalar a41 = L4 * qP(0);
  cc::Scalar a42 = L3 * qP(0);
  cc::Scalar a45 = L5 * qP(0);
  cc::Scalar b1 = (s2s1 * cos(q(2)) + s1 * s3 * cos(q(1))) * s4 + ((-s1 * cos(q(1)) * cos(q(2))) * c4 + c4 * s3 * s2s1);
  cc::Scalar b2 = (c4 * s2 * cos(q(2)) + c4 * s3 * cos(q(1)) + s4 * cos(q(1)) * cos(q(2)) - s2 * s3 * s4) * c1;
  cc::Scalar b3 = (s2s1 * cos(q(2)) + s1 * s3 * cos(q(1))) * c4s5 + ((s1 * s5 * cos(q(1)) * cos(q(2))) * s4 + (-s5) * s3 * s4 * s2s1);
  cc::Scalar b4 = (cos(q(1)) * cos(q(2)) - s2 * s3) * c1;
  cc::Scalar b5 = (c4s5 * s2 * s3 - c4s5 * cos(q(1)) * cos(q(2)) + s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1))) * c1 + c5 * s1;
  cc::Scalar b6 = (s4 * s2s1 * cos(q(2)) + c4 * s3 * s2s1 + s1 * s3 * s4 * cos(q(1)) - c4 * s1 * cos(q(1)) * cos(q(2))) * c5 + c1 * s5;
  cc::Scalar b7 = s2s1 * cos(q(2)) + s1 * s3 * cos(q(1));
  cc::Scalar b8 = (c4 * c5 * s2 * cos(q(2)) - c5 * s2 * s3 * s4 + c4 * c5 * s3 * cos(q(1)) + c5 * s4 * cos(q(1)) * cos(q(2))) * c1;
  cc::Scalar b9 = (c4s5 * s2 * s3 - c4s5 * cos(q(1)) * cos(q(2)) + s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1))) * c1;
  cc::Scalar b10 = (c4 * s2 * s5 * cos(q(2)) - s2 * s3 * s4 * s5 + c4 * s3 * s5 * cos(q(1)) + s4 * s5 * cos(q(1)) * cos(q(2))) * c1;
  cc::Scalar b11 = (s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - c4 * cos(q(1)) * cos(q(2)) + c4 * s2 * s3) * c1;
  cc::Scalar b12 = (c4 * s3 * cos(q(1)) + s4 * cos(q(1)) * cos(q(2)) - s2 * s3 * s4) * s1 + (s2s1 * cos(q(2))) * c4;
  cc::Scalar b13 = (cos(q(1)) * cos(q(2)) - s2 * s3) * s1;
  cc::Scalar b14 = (c4 * c5 * s2 * s3 + c5 * s2 * s4 * cos(q(2)) + c5 * s3 * s4 * cos(q(1)) - c4 * c5 * cos(q(1)) * cos(q(2))) * c1 - s1 * s5;
  cc::Scalar b15 = (s4 * s2s1 * cos(q(2)) + c4 * s3 * s2s1 + s1 * s3 * s4 * cos(q(1)) - c4 * s1 * cos(q(1)) * cos(q(2))) * s5 - c1 * c5;
  cc::Scalar b16 = (s2 * cos(q(2)) + s3 * cos(q(1))) * c1;
  cc::Scalar b17 = (s4 * s2s1 * cos(q(2)) + c4 * s3 * s2s1 + s1 * s3 * s4 * cos(q(1)) - c4 * s1 * cos(q(1)) * cos(q(2))) * s5;
  cc::Scalar b18 = (s3 * cos(q(1))) * c4 * c5 * s1 + (s2s1 * cos(q(2))) * c4 * c5 + (cos(q(1)) * cos(q(2)) - s2 * s3) * c5 * s1 * s4;
  cc::Scalar b19 = (s2 * s4 * cos(q(2)) + s3 * s4 * cos(q(1)) - c4 * cos(q(1)) * cos(q(2)) + c4 * s2 * s3) * c5;
  cc::Scalar b20 = (c4 * s3 + s4 * cos(q(2))) * s2 + (cos(q(1)) * s3 * s4 + (-cos(q(1)) * cos(q(2))) * c4);
  cc::Scalar b21 = (s2 * cos(q(2)) + s3 * cos(q(1))) * c4s5 + ((s5 * cos(q(1)) * cos(q(2))) * s4 + (-s5) * s2 * s3 * s4);
  cc::Scalar b22 = s2 * cos(q(2)) + s3 * cos(q(1));
  cc::Scalar d1 = (qP(1) + qP(2) + qP(3)) * L5;
  cc::Scalar d2 = (qP(1) + qP(2) + qP(3)) * L12;
  cc::Scalar d3 = (qP(1) + qP(2)) * L3;
  cc::Scalar d4 = qP(1) + qP(2) + qP(3);

  Jcm6_0p(0, 0) = b1 * d1 - a40 * b6 - a42 * b4 - a45 * b2 - a39 * b5 - b3 * d2 + b7 * d3 - a41 * s1 + a37 * s2s1 - a38 * c1 * cos(q(1));
  Jcm6_0p(0, 1) = a45 * b1 - a39 * b3 + a40 * b8 + a42 * b7 - b2 * d1 - b4 * d3 - b9 * d2 + a38 * s2s1 - a37 * c1 * cos(q(1));
  Jcm6_0p(0, 2) = a45 * b1 - a39 * b3 + a40 * b8 + a42 * b7 - b2 * d1 - b4 * d3 - b9 * d2;
  Jcm6_0p(0, 3) = a45 * b1 - a39 * b3 + a40 * b8 - b2 * d1 - b9 * d2;
  Jcm6_0p(0, 4) = b8 * d2 - a40 * b5 - a39 * b6;
  Jcm6_0p(1, 0) = (a41 - a37 * s2) * c1 + (a40 * b14 - a39 * b15 - a42 * b13 - a45 * b12 + b10 * d2 - b11 * d1 - b16 * d3 - a38 * s1 * cos(q(1)));
  Jcm6_0p(1, 1) = a39 * b10 - a45 * b11 + a40 * b18 - a42 * b16 - b12 * d1 - b13 * d3 - b17 * d2 - a37 * s1 * cos(q(1)) - a38 * c1 * s2;
  Jcm6_0p(1, 2) = a39 * b10 - a45 * b11 + a40 * b18 - a42 * b16 - b12 * d1 - b13 * d3 - b17 * d2;
  Jcm6_0p(1, 3) = a39 * b10 - a45 * b11 + a40 * b18 - b12 * d1 - b17 * d2;
  Jcm6_0p(1, 4) = a39 * b14 - a40 * b15 + b18 * d2;
  Jcm6_0p(2, 1) = a40 * b19 - b20 * d1 + b21 * d2 - b22 * d3 - a37 * s2;
  Jcm6_0p(2, 2) = a40 * b19 - b20 * d1 + b21 * d2 - b22 * d3;
  Jcm6_0p(2, 3) = a40 * b19 - b20 * d1 + b21 * d2;
  Jcm6_0p(2, 4) = a40 * b21 + b19 * d2;
  Jcm6_0p(3, 1) = c1 * qP(0);
  Jcm6_0p(3, 2) = c1 * qP(0);
  Jcm6_0p(3, 3) = c1 * qP(0);
  Jcm6_0p(3, 4) = -b11 * d4 - b12 * qP(0);
  Jcm6_0p(3, 5) = b10 * d4 - b15 * qP(0) + b14 * qP(4);
  Jcm6_0p(4, 1) = qP(0) * s1;
  Jcm6_0p(4, 2) = qP(0) * s1;
  Jcm6_0p(4, 3) = qP(0) * s1;
  Jcm6_0p(4, 4) = b2 * qP(0) - b1 * d4;
  Jcm6_0p(4, 5) = b3 * d4 + b5 * qP(0) + b6 * qP(4);
  Jcm6_0p(5, 4) = (c4 * s2 * cos(q(2)) + c4 * s3 * cos(q(1)) + s4 * cos(q(1)) * cos(q(2)) - s2 * s3 * s4) * d4;
  Jcm6_0p(5, 5) = (d4 * s2 * s3 * s5 - c5 * qP(4) * s2 * cos(q(2)) - c5 * qP(4) * s3 * cos(q(1)) - d4 * s5 * cos(q(1)) * cos(q(2))) * c4 + ((s4 * s5 * cos(q(1))) * d4 * s3 + (s4 * s5 * cos(q(2))) * d4 * s2 + (c5 * qP(4) * s4) * s2 * s3 - c5 * qP(4) * s4 * cos(q(1)) * cos(q(2)));
}

#endif