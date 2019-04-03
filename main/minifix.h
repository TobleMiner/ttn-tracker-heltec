#pragma once

#include <stdint.h>

typedef struct {
  uint32_t value:24;
} fix24;

fix24 double_to_fix24(double d);

double fix24_to_double(fix24 fix);
