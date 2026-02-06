#ifndef TRANSFORMATIONS_H_
#define TRANSFORMATIONS_H_

#include <ur_model/ur_model.h>

using namespace ur;

void URModel::matrix_T1_0(cc::HomogeneousTransformation &T1_0,
                            const cc::JointPosition &q) const
{

  T1_0(0, 0) = cos(q(0));
  T1_0(0, 1) = 0;
  T1_0(0, 2) = sin(q(0));
  T1_0(0, 3) = 0;
  T1_0(1, 0) = sin(q(0));
  T1_0(1, 1) = 0;
  T1_0(1, 2) = -(cos(q(0)));
  T1_0(1, 3) = 0;
  T1_0(2, 0) = 0;
  T1_0(2, 1) = 1;
  T1_0(2, 2) = 0;
  T1_0(2, 3) = L1;
  T1_0(3, 0) = 0;
  T1_0(3, 1) = 0;
  T1_0(3, 2) = 0;
  T1_0(3, 3) = 1;
}

void URModel::matrix_T2_0(cc::HomogeneousTransformation &T2_0,
                            const cc::JointPosition &q) const
{

  T2_0(0, 0) = (cos(q(1))) * (cos(q(0)));
  T2_0(0, 1) = -((cos(q(0))) * (sin(q(1))));
  T2_0(0, 2) = sin(q(0));
  T2_0(0, 3) = L2 * ((cos(q(1))) * (cos(q(0))));
  T2_0(1, 0) = (cos(q(1))) * (sin(q(0)));
  T2_0(1, 1) = -((sin(q(1))) * (sin(q(0))));
  T2_0(1, 2) = -(cos(q(0)));
  T2_0(1, 3) = L2 * ((cos(q(1))) * (sin(q(0))));
  T2_0(2, 0) = sin(q(1));
  T2_0(2, 1) = cos(q(1));
  T2_0(2, 2) = 0;
  T2_0(2, 3) = L1 + L2 * sin(q(1));
  T2_0(3, 0) = 0;
  T2_0(3, 1) = 0;
  T2_0(3, 2) = 0;
  T2_0(3, 3) = 1;
}

void URModel::matrix_T3_0(cc::HomogeneousTransformation &T3_0,
                            const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));

  T3_0(0, 0) = cos(q(0)) * cos(q(1)) * cos(q(2)) - s2 * cos(q(0)) * sin(q(2));
  T3_0(0, 1) = -s2 * cos(q(0)) * cos(q(2)) - cos(q(0)) * cos(q(1)) * sin(q(2));
  T3_0(0, 2) = s1;
  T3_0(0, 3) = (cos(q(0)) * cos(q(1)) * cos(q(2)) - s2 * cos(q(0)) * sin(q(2))) * L3 + L2 * cos(q(0)) * cos(q(1));
  T3_0(1, 0) = (cos(q(1)) * cos(q(2)) - s2 * sin(q(2))) * s1;
  T3_0(1, 1) = (-cos(q(1)) * sin(q(2)) - s2 * cos(q(2))) * s1;
  T3_0(1, 2) = -(cos(q(0)));
  T3_0(1, 3) = (cos(q(1)) * cos(q(2)) - s2 * sin(q(2))) * L3 * s1 + (L2 * cos(q(1))) * s1;
  T3_0(2, 0) = cos(q(1)) * sin(q(2)) + s2 * cos(q(2));
  T3_0(2, 1) = cos(q(1)) * cos(q(2)) - s2 * sin(q(2));
  T3_0(2, 2) = 0;
  T3_0(2, 3) = (L2 + L3 * cos(q(2))) * s2 + (L1 + (cos(q(1)) * sin(q(2))) * L3);
  T3_0(3, 0) = 0;
  T3_0(3, 1) = 0;
  T3_0(3, 2) = 0;
  T3_0(3, 3) = 1;
}

void URModel::matrix_T4_0(cc::HomogeneousTransformation &T4_0,
                            const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s3 = sin(q(2));
  cc::Scalar c2s1 = (cos(q(1))) * s1;

  T4_0(0, 0) = (-s2 * cos(q(0)) * cos(q(3)) - cos(q(0)) * cos(q(1)) * sin(q(3))) * s3 + ((-cos(q(0)) * cos(q(2)) * sin(q(3))) * s2 + cos(q(0)) * cos(q(1)) * cos(q(2)) * cos(q(3)));
  T4_0(0, 1) = s1;
  T4_0(0, 2) = (cos(q(0)) * cos(q(2)) * cos(q(3)) - s3 * cos(q(0)) * sin(q(3))) * s2 + ((cos(q(0)) * cos(q(1)) * cos(q(3))) * s3 + cos(q(0)) * cos(q(1)) * cos(q(2)) * sin(q(3)));
  T4_0(0, 3) = (cos(q(0)) * cos(q(1)) * cos(q(2)) - s2 * s3 * cos(q(0))) * L3 + (L4 * s1 + L2 * cos(q(0)) * cos(q(1)));
  T4_0(1, 0) = (cos(q(2)) * cos(q(3)) - s3 * sin(q(3))) * c2s1 + ((-cos(q(2)) * sin(q(3))) * s1 * s2 + (-cos(q(3))) * s1 * s2 * s3);
  T4_0(1, 1) = -(cos(q(0)));
  T4_0(1, 2) = (cos(q(2)) * sin(q(3)) + s3 * cos(q(3))) * c2s1 + ((cos(q(2)) * cos(q(3))) * s1 * s2 + (-sin(q(3))) * s1 * s2 * s3);
  T4_0(1, 3) = (L2 + L3 * cos(q(2))) * c2s1 + ((-s1 * s2 * s3) * L3 - L4 * cos(q(0)));
  T4_0(2, 0) = (cos(q(2)) * cos(q(3)) - s3 * sin(q(3))) * s2 + ((cos(q(1)) * cos(q(3))) * s3 + cos(q(1)) * cos(q(2)) * sin(q(3)));
  T4_0(2, 1) = 0;
  T4_0(2, 2) = (cos(q(2)) * sin(q(3)) + s3 * cos(q(3))) * s2 + ((cos(q(1)) * sin(q(3))) * s3 - cos(q(1)) * cos(q(2)) * cos(q(3)));
  T4_0(2, 3) = (L2 + L3 * cos(q(2))) * s2 + (L1 + (s3 * cos(q(1))) * L3);
  T4_0(3, 0) = 0;
  T4_0(3, 1) = 0;
  T4_0(3, 2) = 0;
  T4_0(3, 3) = 1;
}

void URModel::matrix_T5_0(cc::HomogeneousTransformation &T5_0,
                            const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s3 = sin(q(2));
  cc::Scalar s5 = sin(q(4));
  cc::Scalar c3 = cos(q(2));
  cc::Scalar c2s1 = (cos(q(1))) * s1;

  T5_0(0, 0) = (cos(q(0)) * cos(q(1)) * cos(q(3)) * cos(q(4)) - s2 * cos(q(0)) * cos(q(4)) * sin(q(3))) * c3 + (s1 * s5 + (-cos(q(0)) * cos(q(1)) * cos(q(4)) * sin(q(3))) * s3 + (-cos(q(0)) * cos(q(3)) * cos(q(4))) * s2 * s3);
  T5_0(0, 1) = (s3 * cos(q(0)) * sin(q(3)) - c3 * cos(q(0)) * cos(q(3))) * s2 + ((-cos(q(0)) * cos(q(1)) * sin(q(3))) * c3 + (-cos(q(0)) * cos(q(1)) * cos(q(3))) * s3);
  T5_0(0, 2) = (s3 * cos(q(0)) * cos(q(1)) * sin(q(3)) - c3 * cos(q(0)) * cos(q(1)) * cos(q(3)) + c3 * s2 * cos(q(0)) * sin(q(3)) + s2 * s3 * cos(q(0)) * cos(q(3))) * s5 + s1 * cos(q(4));
  T5_0(0, 3) = (c3 * cos(q(0)) * cos(q(1)) * sin(q(3)) + s3 * cos(q(0)) * cos(q(1)) * cos(q(3)) + c3 * s2 * cos(q(0)) * cos(q(3)) - s2 * s3 * cos(q(0)) * sin(q(3))) * L5 + ((L3 * cos(q(0)) * cos(q(1))) * c3 + L4 * s1 + L2 * cos(q(0)) * cos(q(1)) + (-L3 * cos(q(0))) * s2 * s3);
  T5_0(1, 0) = (c3 * cos(q(1)) * cos(q(3)) * cos(q(4)) - c3 * s2 * cos(q(4)) * sin(q(3)) - s2 * s3 * cos(q(3)) * cos(q(4))) * s1 + ((-c2s1 * cos(q(4)) * sin(q(3))) * s3 - s5 * cos(q(0)));
  T5_0(1, 1) = (s2 * s3 * sin(q(3)) - c3 * s2 * cos(q(3)) - c3 * cos(q(1)) * sin(q(3))) * s1 + (-c2s1 * cos(q(3))) * s3;
  T5_0(1, 2) = (s2 * sin(q(3)) - cos(q(1)) * cos(q(3))) * c3 * s1 * s5 + (s2 * cos(q(3))) * s1 * s3 * s5 + (c2s1 * sin(q(3))) * s3 * s5 - cos(q(0)) * cos(q(4));
  T5_0(1, 3) = (L3 * c3 * cos(q(1)) - L3 * s2 * s3 + L5 * c3 * s2 * cos(q(3)) - L5 * s2 * s3 * sin(q(3)) + L5 * c3 * cos(q(1)) * sin(q(3))) * s1 + (L2 * c2s1 - L4 * cos(q(0)) + (c2s1 * cos(q(3))) * L5 * s3);
  T5_0(2, 0) = (s2 * cos(q(3)) * cos(q(4)) + cos(q(1)) * cos(q(4)) * sin(q(3))) * c3 + ((cos(q(1)) * cos(q(3)) * cos(q(4))) * s3 + (-cos(q(4)) * sin(q(3))) * s2 * s3);
  T5_0(2, 1) = (cos(q(1)) * cos(q(3)) - s2 * sin(q(3))) * c3 + ((-cos(q(1)) * sin(q(3))) * s3 + (-cos(q(3))) * s2 * s3);
  T5_0(2, 2) = (s2 * s3 * sin(q(3)) - c3 * s2 * cos(q(3)) - c3 * cos(q(1)) * sin(q(3)) - s3 * cos(q(1)) * cos(q(3))) * s5;
  T5_0(2, 3) = (L2 + L3 * c3 + L5 * c3 * sin(q(3)) + L5 * s3 * cos(q(3))) * s2 + (L1 + (L3 * cos(q(1))) * s3 + (cos(q(1)) * sin(q(3))) * L5 * s3 + (-cos(q(1)) * cos(q(3))) * L5 * c3);
  T5_0(3, 0) = 0;
  T5_0(3, 1) = 0;
  T5_0(3, 2) = 0;
  T5_0(3, 3) = 1;
}

void URModel::matrix_T6_0(cc::HomogeneousTransformation &T6_0,
                            const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s3 = sin(q(2));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar s5 = sin(q(4));
  cc::Scalar s6 = sin(q(5));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c4 = cos(q(3));
  cc::Scalar c3s2 = (cos(q(2))) * s2;
  cc::Scalar c2s3 = (cos(q(1))) * s3;
  cc::Scalar c2s1 = (cos(q(1))) * s1;
  cc::Scalar c4s1 = c4 * s1;

  T6_0(0, 0) = (s2 * s3 * s4 * s6 - c4 * s2 * s6 * cos(q(2)) - c4 * s3 * s6 * cos(q(1)) - s4 * s6 * cos(q(1)) * cos(q(2)) + c4 * cos(q(1)) * cos(q(2)) * cos(q(4)) * cos(q(5)) - c4 * s2 * s3 * cos(q(4)) * cos(q(5)) - s2 * s4 * cos(q(2)) * cos(q(4)) * cos(q(5)) - s3 * s4 * cos(q(1)) * cos(q(4)) * cos(q(5))) * c1 + s1 * s5 * cos(q(5));
  T6_0(0, 1) = (s2 * s3 * s4 * cos(q(5)) - s4 * cos(q(1)) * cos(q(2)) * cos(q(5)) - c4 * c3s2 * cos(q(5)) - c4 * s3 * cos(q(1)) * cos(q(5)) + s2 * s4 * s6 * cos(q(2)) * cos(q(4)) + s3 * s4 * s6 * cos(q(1)) * cos(q(4)) - c4 * s6 * cos(q(1)) * cos(q(2)) * cos(q(4)) + c4 * s2 * s3 * s6 * cos(q(4))) * c1 + (-s1 * s5) * s6;
  T6_0(0, 2) = (c4 * s2 * s3 * s5 + s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1)) - c4 * s5 * cos(q(1)) * cos(q(2))) * c1 + s1 * cos(q(4));
  T6_0(0, 3) = (c2s3 + c3s2) * L5 * c1 * c4 + (s4 * cos(q(1)) * cos(q(2)) - s2 * s3 * s4) * L5 * c1 + (s2 * s3 * s5 - s5 * cos(q(1)) * cos(q(2))) * L6 * c1 * c4 + (s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1))) * L6 * c1 + (s1 * cos(q(4))) * L6 + (L2 * cos(q(1)) + L3 * cos(q(1)) * cos(q(2)) - L3 * s2 * s3) * c1 + L4 * s1;
  T6_0(1, 0) = (-c2s3 - c3s2) * c4 * s1 * s6 + (-s2 * s3 * cos(q(4)) * cos(q(5))) * c4 * s1 + (c2s1 * cos(q(2)) * cos(q(4)) * cos(q(5))) * c4 + (s2 * s3) * s1 * s4 * s6 + (-c3s2 * cos(q(4)) * cos(q(5))) * s1 * s4 + (-c2s1 * cos(q(2))) * s4 * s6 + (-c2s1 * s3 * cos(q(4)) * cos(q(5))) * s4 - c1 * s5 * cos(q(5));
  T6_0(1, 1) = (c1 * s5 + c2s1 * s3 * s4 * cos(q(4)) + c3s2 * s1 * s4 * cos(q(4)) - c4 * c2s1 * cos(q(2)) * cos(q(4)) + c4 * s1 * s2 * s3 * cos(q(4))) * s6 + ((-c4s1 * cos(q(1)) * cos(q(5))) * s3 + (s1 * s2 * cos(q(5))) * s3 * s4 - c3s2 * c4s1 * cos(q(5)) + (-cos(q(2)) * cos(q(5))) * c2s1 * s4);
  T6_0(1, 2) = (c2s1 * s3 * s4 + c3s2 * s1 * s4 + c4s1 * s2 * s3 - c4 * c2s1 * cos(q(2))) * s5 - c1 * cos(q(4));
  T6_0(1, 3) = (s4 * cos(q(2))) * L5 * c2s1 + (-s1 * s2 * s4) * L5 * s3 + (c2s3 * c4s1 + c3s2 * c4s1) * L5 + (s4 * s5) * L6 * c2s1 * s3 + (-c4 * s5 * cos(q(2))) * L6 * c2s1 + (c4s1 * s2 * s5) * L6 * s3 + (c3s2 * s1 * s4 * s5 - c1 * cos(q(4))) * L6 + (L2 + L3 * cos(q(2))) * c2s1 + (-L3 * s1 * s2) * s3 - L4 * c1;
  T6_0(2, 0) = (-s2) * c4 * s3 * s6 + (cos(q(1)) * cos(q(2))) * c4 * s6 + (c2s3 * cos(q(4)) * cos(q(5)) + c3s2 * cos(q(4)) * cos(q(5))) * c4 + (-cos(q(1))) * s3 * s4 * s6 + (-s2 * cos(q(4)) * cos(q(5))) * s3 * s4 + (-c3s2) * s4 * s6 + (cos(q(1)) * cos(q(2)) * cos(q(4)) * cos(q(5))) * s4;
  T6_0(2, 1) = (-s2 * cos(q(5))) * c4 * s3 + (-c2s3 * cos(q(4)) - c3s2 * cos(q(4))) * c4 * s6 + (cos(q(1)) * cos(q(2)) * cos(q(5))) * c4 + (s2 * cos(q(4))) * s3 * s4 * s6 + (-cos(q(1)) * cos(q(5))) * s3 * s4 + (-cos(q(1)) * cos(q(2)) * cos(q(4))) * s4 * s6 + (-c3s2 * cos(q(5))) * s4;
  T6_0(2, 2) = (-c2s3 - c3s2) * c4 * s5 + (s2 * s3 - cos(q(1)) * cos(q(2))) * s4 * s5;
  T6_0(2, 3) = (s2 * s3 - cos(q(1)) * cos(q(2))) * L5 * c4 + (c2s3 + c3s2) * L5 * s4 + (-c2s3 * s5 - c3s2 * s5) * L6 * c4 + (s2 * s3 * s5 - s5 * cos(q(1)) * cos(q(2))) * L6 * s4 + (L1 + L3 * c2s3 + L3 * c3s2 + L2 * s2);
  T6_0(3, 0) = 0;
  T6_0(3, 1) = 0;
  T6_0(3, 2) = 0;
  T6_0(3, 3) = 1;
}

void URModel::matrix_Tcm1_0(cc::HomogeneousTransformation &Tcm1_0,
                              const cc::JointPosition &q) const
{

  Tcm1_0(0, 0) = cos(q(0));
  Tcm1_0(0, 1) = -(sin(q(0)));
  Tcm1_0(0, 2) = 0;
  Tcm1_0(0, 3) = 0;
  Tcm1_0(1, 0) = sin(q(0));
  Tcm1_0(1, 1) = cos(q(0));
  Tcm1_0(1, 2) = 0;
  Tcm1_0(1, 3) = 0;
  Tcm1_0(2, 0) = 0;
  Tcm1_0(2, 1) = 0;
  Tcm1_0(2, 2) = 1;
  Tcm1_0(2, 3) = L7;
  Tcm1_0(3, 0) = 0;
  Tcm1_0(3, 1) = 0;
  Tcm1_0(3, 2) = 0;
  Tcm1_0(3, 3) = 1;
}

void URModel::matrix_Tcm2_0(cc::HomogeneousTransformation &Tcm2_0,
                              const cc::JointPosition &q) const
{

  Tcm2_0(0, 0) = (cos(q(1))) * (cos(q(0)));
  Tcm2_0(0, 1) = -((cos(q(0))) * (sin(q(1))));
  Tcm2_0(0, 2) = sin(q(0));
  Tcm2_0(0, 3) = L8 * ((cos(q(1))) * (cos(q(0))));
  Tcm2_0(1, 0) = (cos(q(1))) * (sin(q(0)));
  Tcm2_0(1, 1) = -((sin(q(1))) * (sin(q(0))));
  Tcm2_0(1, 2) = -(cos(q(0)));
  Tcm2_0(1, 3) = L8 * ((cos(q(1))) * (sin(q(0))));
  Tcm2_0(2, 0) = sin(q(1));
  Tcm2_0(2, 1) = cos(q(1));
  Tcm2_0(2, 2) = 0;
  Tcm2_0(2, 3) = L1 + L8 * sin(q(1));
  Tcm2_0(3, 0) = 0;
  Tcm2_0(3, 1) = 0;
  Tcm2_0(3, 2) = 0;
  Tcm2_0(3, 3) = 1;
}

void URModel::matrix_Tcm3_0(cc::HomogeneousTransformation &Tcm3_0,
                              const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));

  Tcm3_0(0, 0) = cos(q(0)) * cos(q(1)) * cos(q(2)) - s2 * cos(q(0)) * sin(q(2));
  Tcm3_0(0, 1) = -s2 * cos(q(0)) * cos(q(2)) - cos(q(0)) * cos(q(1)) * sin(q(2));
  Tcm3_0(0, 2) = s1;
  Tcm3_0(0, 3) = (cos(q(0)) * cos(q(1)) * cos(q(2)) - s2 * cos(q(0)) * sin(q(2))) * L9 + L2 * cos(q(0)) * cos(q(1));
  Tcm3_0(1, 0) = (cos(q(1)) * cos(q(2)) - s2 * sin(q(2))) * s1;
  Tcm3_0(1, 1) = (-cos(q(1)) * sin(q(2)) - s2 * cos(q(2))) * s1;
  Tcm3_0(1, 2) = -(cos(q(0)));
  Tcm3_0(1, 3) = (cos(q(1)) * cos(q(2)) - s2 * sin(q(2))) * L9 * s1 + (L2 * cos(q(1))) * s1;
  Tcm3_0(2, 0) = cos(q(1)) * sin(q(2)) + s2 * cos(q(2));
  Tcm3_0(2, 1) = cos(q(1)) * cos(q(2)) - s2 * sin(q(2));
  Tcm3_0(2, 2) = 0;
  Tcm3_0(2, 3) = (L2 + L9 * cos(q(2))) * s2 + (L1 + (cos(q(1)) * sin(q(2))) * L9);
  Tcm3_0(3, 0) = 0;
  Tcm3_0(3, 1) = 0;
  Tcm3_0(3, 2) = 0;
  Tcm3_0(3, 3) = 1;
}

void URModel::matrix_Tcm4_0(cc::HomogeneousTransformation &Tcm4_0,
                              const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s3 = sin(q(2));
  cc::Scalar c2s1 = (cos(q(1))) * s1;

  Tcm4_0(0, 0) = (-s2 * cos(q(0)) * cos(q(3)) - cos(q(0)) * cos(q(1)) * sin(q(3))) * s3 + ((-cos(q(0)) * cos(q(2)) * sin(q(3))) * s2 + cos(q(0)) * cos(q(1)) * cos(q(2)) * cos(q(3)));
  Tcm4_0(0, 1) = (s3 * cos(q(0)) * sin(q(3)) - cos(q(0)) * cos(q(2)) * cos(q(3))) * s2 + ((-cos(q(0)) * cos(q(1)) * cos(q(3))) * s3 - cos(q(0)) * cos(q(1)) * cos(q(2)) * sin(q(3)));
  Tcm4_0(0, 2) = s1;
  Tcm4_0(0, 3) = (cos(q(0)) * cos(q(1)) * cos(q(2)) - s2 * s3 * cos(q(0))) * L3 + (L10 * s1 + L2 * cos(q(0)) * cos(q(1)));
  Tcm4_0(1, 0) = (cos(q(2)) * cos(q(3)) - s3 * sin(q(3))) * c2s1 + ((-cos(q(2)) * sin(q(3))) * s1 * s2 + (-cos(q(3))) * s1 * s2 * s3);
  Tcm4_0(1, 1) = (s2 * s3 * sin(q(3)) - s2 * cos(q(2)) * cos(q(3))) * s1 + ((-cos(q(2)) * sin(q(3))) * c2s1 + (-cos(q(3))) * c2s1 * s3);
  Tcm4_0(1, 2) = -(cos(q(0)));
  Tcm4_0(1, 3) = (L2 + L3 * cos(q(2))) * c2s1 + ((-s1 * s2 * s3) * L3 - L10 * cos(q(0)));
  Tcm4_0(2, 0) = (cos(q(2)) * cos(q(3)) - s3 * sin(q(3))) * s2 + ((cos(q(1)) * cos(q(3))) * s3 + cos(q(1)) * cos(q(2)) * sin(q(3)));
  Tcm4_0(2, 1) = (-cos(q(2)) * sin(q(3)) - s3 * cos(q(3))) * s2 + (cos(q(1)) * cos(q(2)) * cos(q(3)) + (-cos(q(1)) * sin(q(3))) * s3);
  Tcm4_0(2, 2) = 0;
  Tcm4_0(2, 3) = (L2 + L3 * cos(q(2))) * s2 + (L1 + (s3 * cos(q(1))) * L3);
  Tcm4_0(3, 0) = 0;
  Tcm4_0(3, 1) = 0;
  Tcm4_0(3, 2) = 0;
  Tcm4_0(3, 3) = 1;
}

void URModel::matrix_Tcm5_0(cc::HomogeneousTransformation &Tcm5_0,
                              const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s3 = sin(q(2));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar s5 = sin(q(4));
  cc::Scalar c2s1 = (cos(q(1))) * s1;

  Tcm5_0(0, 0) = (-s3 * cos(q(0)) * cos(q(3)) * cos(q(4)) - s4 * cos(q(0)) * cos(q(2)) * cos(q(4))) * s2 + (s1 * s5 + (-cos(q(0)) * cos(q(1)) * cos(q(4))) * s3 * s4 + cos(q(0)) * cos(q(1)) * cos(q(2)) * cos(q(3)) * cos(q(4)));
  Tcm5_0(0, 1) = (s2 * s3 * cos(q(0)) * cos(q(3)) - cos(q(0)) * cos(q(1)) * cos(q(2)) * cos(q(3)) + s2 * s4 * cos(q(0)) * cos(q(2)) + s3 * s4 * cos(q(0)) * cos(q(1))) * s5 + s1 * cos(q(4));
  Tcm5_0(0, 2) = (cos(q(0)) * cos(q(2)) * cos(q(3)) - s3 * s4 * cos(q(0))) * s2 + ((cos(q(0)) * cos(q(1)) * cos(q(2))) * s4 + (cos(q(0)) * cos(q(1)) * cos(q(3))) * s3);
  Tcm5_0(0, 3) = (s2 * cos(q(0)) * cos(q(2)) * cos(q(3)) + s3 * cos(q(0)) * cos(q(1)) * cos(q(3)) + s4 * cos(q(0)) * cos(q(1)) * cos(q(2)) - s2 * s3 * s4 * cos(q(0))) * L11 + (L4 * s1 + (cos(q(0)) * cos(q(1)) * cos(q(2))) * L3 + L2 * cos(q(0)) * cos(q(1)) + (-cos(q(0))) * L3 * s2 * s3);
  Tcm5_0(1, 0) = (cos(q(1)) * cos(q(2)) * cos(q(3)) * cos(q(4)) - s2 * s3 * cos(q(3)) * cos(q(4)) - s2 * s4 * cos(q(2)) * cos(q(4))) * s1 + ((-c2s1 * cos(q(4))) * s3 * s4 - s5 * cos(q(0)));
  Tcm5_0(1, 1) = (c2s1 * s3 * s4 - s1 * cos(q(1)) * cos(q(2)) * cos(q(3)) + s1 * s2 * s3 * cos(q(3)) + s1 * s2 * s4 * cos(q(2))) * s5 - cos(q(0)) * cos(q(4));
  Tcm5_0(1, 2) = (s2 * cos(q(2)) * cos(q(3)) + s4 * cos(q(1)) * cos(q(2)) - s2 * s3 * s4) * s1 + (c2s1 * cos(q(3))) * s3;
  Tcm5_0(1, 3) = (L3 * cos(q(1)) * cos(q(2)) - L3 * s2 * s3 - L11 * s2 * s3 * s4 + L11 * s2 * cos(q(2)) * cos(q(3)) + L11 * s4 * cos(q(1)) * cos(q(2))) * s1 + (L2 * c2s1 - L4 * cos(q(0)) + (c2s1 * cos(q(3))) * L11 * s3);
  Tcm5_0(2, 0) = (cos(q(2)) * cos(q(3)) * cos(q(4)) - s3 * s4 * cos(q(4))) * s2 + ((cos(q(1)) * cos(q(2)) * cos(q(4))) * s4 + (cos(q(1)) * cos(q(3)) * cos(q(4))) * s3);
  Tcm5_0(2, 1) = (s2 * s3 * s4 - s3 * cos(q(1)) * cos(q(3)) - s4 * cos(q(1)) * cos(q(2)) - s2 * cos(q(2)) * cos(q(3))) * s5;
  Tcm5_0(2, 2) = (s3 * cos(q(3)) + s4 * cos(q(2))) * s2 + (cos(q(1)) * s3 * s4 - cos(q(1)) * cos(q(2)) * cos(q(3)));
  Tcm5_0(2, 3) = (L2 + L3 * cos(q(2)) + L11 * s3 * cos(q(3)) + L11 * s4 * cos(q(2))) * s2 + (L1 + (-cos(q(1)) * cos(q(2)) * cos(q(3))) * L11 + (s4 * cos(q(1))) * L11 * s3 + cos(q(1)) * L3 * s3);
  Tcm5_0(3, 0) = 0;
  Tcm5_0(3, 1) = 0;
  Tcm5_0(3, 2) = 0;
  Tcm5_0(3, 3) = 1;
}

void URModel::matrix_Tcm6_0(cc::HomogeneousTransformation &Tcm6_0,
                              const cc::JointPosition &q) const
{
  cc::Scalar s1 = sin(q(0));
  cc::Scalar s2 = sin(q(1));
  cc::Scalar s3 = sin(q(2));
  cc::Scalar s4 = sin(q(3));
  cc::Scalar s5 = sin(q(4));
  cc::Scalar s6 = sin(q(5));
  cc::Scalar c1 = cos(q(0));
  cc::Scalar c4 = cos(q(3));
  cc::Scalar c3s2 = (cos(q(2))) * s2;
  cc::Scalar c2s3 = (cos(q(1))) * s3;
  cc::Scalar c2s1 = (cos(q(1))) * s1;
  cc::Scalar c4s1 = c4 * s1;

  Tcm6_0(0, 0) = (s2 * s3 * s4 * s6 - c4 * s2 * s6 * cos(q(2)) - c4 * s3 * s6 * cos(q(1)) - s4 * s6 * cos(q(1)) * cos(q(2)) + c4 * cos(q(1)) * cos(q(2)) * cos(q(4)) * cos(q(5)) - c4 * s2 * s3 * cos(q(4)) * cos(q(5)) - s2 * s4 * cos(q(2)) * cos(q(4)) * cos(q(5)) - s3 * s4 * cos(q(1)) * cos(q(4)) * cos(q(5))) * c1 + s1 * s5 * cos(q(5));
  Tcm6_0(0, 1) = (s2 * s3 * s4 * cos(q(5)) - s4 * cos(q(1)) * cos(q(2)) * cos(q(5)) - c4 * c3s2 * cos(q(5)) - c4 * s3 * cos(q(1)) * cos(q(5)) + s2 * s4 * s6 * cos(q(2)) * cos(q(4)) + s3 * s4 * s6 * cos(q(1)) * cos(q(4)) - c4 * s6 * cos(q(1)) * cos(q(2)) * cos(q(4)) + c4 * s2 * s3 * s6 * cos(q(4))) * c1 + (-s1 * s5) * s6;
  Tcm6_0(0, 2) = (c4 * s2 * s3 * s5 + s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1)) - c4 * s5 * cos(q(1)) * cos(q(2))) * c1 + s1 * cos(q(4));
  Tcm6_0(0, 3) = (c2s3 + c3s2) * L5 * c1 * c4 + (s4 * cos(q(1)) * cos(q(2)) - s2 * s3 * s4) * L5 * c1 + (s2 * s3 * s5 - s5 * cos(q(1)) * cos(q(2))) * L12 * c1 * c4 + (s2 * s4 * s5 * cos(q(2)) + s3 * s4 * s5 * cos(q(1))) * L12 * c1 + (s1 * cos(q(4))) * L12 + (L2 * cos(q(1)) + L3 * cos(q(1)) * cos(q(2)) - L3 * s2 * s3) * c1 + L4 * s1;
  Tcm6_0(1, 0) = (-c2s3 - c3s2) * c4 * s1 * s6 + (-s2 * s3 * cos(q(4)) * cos(q(5))) * c4 * s1 + (c2s1 * cos(q(2)) * cos(q(4)) * cos(q(5))) * c4 + (s2 * s3) * s1 * s4 * s6 + (-c3s2 * cos(q(4)) * cos(q(5))) * s1 * s4 + (-c2s1 * cos(q(2))) * s4 * s6 + (-c2s1 * s3 * cos(q(4)) * cos(q(5))) * s4 - c1 * s5 * cos(q(5));
  Tcm6_0(1, 1) = (c1 * s5 + c2s1 * s3 * s4 * cos(q(4)) + c3s2 * s1 * s4 * cos(q(4)) - c4 * c2s1 * cos(q(2)) * cos(q(4)) + c4 * s1 * s2 * s3 * cos(q(4))) * s6 + ((-c4s1 * cos(q(1)) * cos(q(5))) * s3 + (s1 * s2 * cos(q(5))) * s3 * s4 - c3s2 * c4s1 * cos(q(5)) + (-cos(q(2)) * cos(q(5))) * c2s1 * s4);
  Tcm6_0(1, 2) = (c2s1 * s3 * s4 + c3s2 * s1 * s4 + c4s1 * s2 * s3 - c4 * c2s1 * cos(q(2))) * s5 - c1 * cos(q(4));
  Tcm6_0(1, 3) = (s4 * cos(q(2))) * L5 * c2s1 + (-s1 * s2 * s4) * L5 * s3 + (c2s3 * c4s1 + c3s2 * c4s1) * L5 + (s4 * s5) * L12 * c2s1 * s3 + (-c4 * s5 * cos(q(2))) * L12 * c2s1 + (c4s1 * s2 * s5) * L12 * s3 + (c3s2 * s1 * s4 * s5 - c1 * cos(q(4))) * L12 + (L2 + L3 * cos(q(2))) * c2s1 + (-L3 * s1 * s2) * s3 - L4 * c1;
  Tcm6_0(2, 0) = (-s2) * c4 * s3 * s6 + (cos(q(1)) * cos(q(2))) * c4 * s6 + (c2s3 * cos(q(4)) * cos(q(5)) + c3s2 * cos(q(4)) * cos(q(5))) * c4 + (-cos(q(1))) * s3 * s4 * s6 + (-s2 * cos(q(4)) * cos(q(5))) * s3 * s4 + (-c3s2) * s4 * s6 + (cos(q(1)) * cos(q(2)) * cos(q(4)) * cos(q(5))) * s4;
  Tcm6_0(2, 1) = (-s2 * cos(q(5))) * c4 * s3 + (-c2s3 * cos(q(4)) - c3s2 * cos(q(4))) * c4 * s6 + (cos(q(1)) * cos(q(2)) * cos(q(5))) * c4 + (s2 * cos(q(4))) * s3 * s4 * s6 + (-cos(q(1)) * cos(q(5))) * s3 * s4 + (-cos(q(1)) * cos(q(2)) * cos(q(4))) * s4 * s6 + (-c3s2 * cos(q(5))) * s4;
  Tcm6_0(2, 2) = (-c2s3 - c3s2) * c4 * s5 + (s2 * s3 - cos(q(1)) * cos(q(2))) * s4 * s5;
  Tcm6_0(2, 3) = (s2 * s3 - cos(q(1)) * cos(q(2))) * L5 * c4 + (c2s3 + c3s2) * L5 * s4 + (-c2s3 * s5 - c3s2 * s5) * L12 * c4 + (s2 * s3 * s5 - s5 * cos(q(1)) * cos(q(2))) * L12 * s4 + (L1 + L3 * c2s3 + L3 * c3s2 + L2 * s2);
  Tcm6_0(3, 0) = 0;
  Tcm6_0(3, 1) = 0;
  Tcm6_0(3, 2) = 0;
  Tcm6_0(3, 3) = 1;
}

#endif