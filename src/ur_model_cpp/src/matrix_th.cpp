#ifndef MATRIX_TH_H_
#define MATRIX_TH_H_

#include <ur_model/ur_model.h>

using namespace ur;

void URModel::matrix_th(Parameters &th) const
{
  th(0, 0) = I133;
  th(1, 0) = L8 * m2;
  th(2, 0) = (L8 * L8) * m2;
  th(3, 0) = I211;
  th(4, 0) = I222;
  th(5, 0) = I233;
  th(6, 0) = I212;
  th(7, 0) = I213;
  th(8, 0) = I223;
  th(9, 0) = m3;
  th(10, 0) = L9 * m3;
  th(11, 0) = (L9 * L9) * m3;
  th(12, 0) = I311;
  th(13, 0) = I322;
  th(14, 0) = I333;
  th(15, 0) = I312;
  th(16, 0) = I313;
  th(17, 0) = I323;
  th(18, 0) = m4;
  th(19, 0) = L10 * m4;
  th(20, 0) = (L10 * L10) * m4;
  th(21, 0) = I411;
  th(22, 0) = I422;
  th(23, 0) = I433;
  th(24, 0) = I412;
  th(25, 0) = I413;
  th(26, 0) = I423;
  th(27, 0) = m5;
  th(28, 0) = L11 * m5;
  th(29, 0) = (L11 * L11) * m5;
  th(30, 0) = I511;
  th(31, 0) = I522;
  th(32, 0) = I533;
  th(33, 0) = I512;
  th(34, 0) = I513;
  th(35, 0) = I523;
  th(36, 0) = m6;
  th(37, 0) = L12 * m6;
  th(38, 0) = (L12 * L12) * m6;
  th(39, 0) = I611;
  th(40, 0) = I622;
  th(41, 0) = I633;
  th(42, 0) = I612;
  th(43, 0) = I613;
  th(44, 0) = I623;
}

#endif
