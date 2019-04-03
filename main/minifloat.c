#include "minifloat.h"

ufloat8 float_to_ufloat8(float val) {
  struct float_t f;
  f.f = val;
  if(val < 0.1275) {
    return 0b0000000;
  }
  if(val > 15872) {
    return 0b1111111;
  }
  return (uint8_t)((((((int32_t)f.fval.exponent) - 127) + 2) & 0xf) << 4) | (uint8_t)((f.fval.mantissa >> 19) & 0xf);
}

float ufloat8_to_float(ufloat8 uf) {
  uint16_t mantissa = (uf & 0xf) | (1<<4);
  int8_t exponent = ((uf >> 4) & 0xf) - 2;
  if(exponent == 0) {
    return 1.0;
  }
  exponent -= 4;
  if(exponent >= 0) {
    return (float)(mantissa << exponent);
  } else {
    float f = mantissa;
    while(exponent++ < 0) {
      f /= 2;
    }
    return f;
  }
}
