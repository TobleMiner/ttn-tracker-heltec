#pragma once

#include <stdint.h>

typedef uint8_t ufloat8;

struct float_t {
  union {
    struct {
      uint32_t mantissa:23;
      uint32_t exponent:8;
      uint32_t sign:1;
    } fval;
    float f;
  };
};

ufloat8 float_to_ufloat8(float val);

float ufloat8_to_float(ufloat8 uf);