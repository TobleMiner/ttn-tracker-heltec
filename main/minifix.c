#include <math.h>

#include "minifix.h"

#define BITS_NUM 8
#define BITS_FRAC 15

static const double fp_exp = exp2(BITS_FRAC);

fix24 double_to_fix24(double d) {
  fix24 val;
  val.value = (d < 0) << (BITS_NUM + BITS_FRAC);
  val.value |= (((uint32_t)floor(fabs(d))) & ((1 << BITS_NUM) - 1)) << BITS_FRAC;
  val.value |= ((uint32_t)round((fabs(d) - floor(fabs(d))) * fp_exp)) & ((1 << BITS_FRAC) - 1);
  return val;
};

double fix24_to_double(fix24 fix) {
  double d = (fix.value >> BITS_FRAC) & ((1 << BITS_NUM) - 1);
  d += ((double)(fix.value & ((1 << BITS_FRAC) - 1))) / fp_exp;
  if(fix.value & (1 << (BITS_NUM + BITS_FRAC))) {
    d = -d;
  }
  return d;
}
