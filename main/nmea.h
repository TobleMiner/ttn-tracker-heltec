#pragma once

#include <stdbool.h>

#include "time.h"

enum {
  NMEA_OK = 0,
  NMEA_CSUM_INVALID,
  NMEA_UNKNOWN_MESSAGE,
  NMEA_SYNTAX_ERROR,
};

struct coordinate {
  double deg;
  char dir;
};

struct nmea {
  struct {
    struct coordinate lat;
    struct coordinate lng;
    double hdop;
    double alt_msl;
    unsigned int quality;
    unsigned int num_sats; // Number used for fix
    struct timeval time; // Time of fix message
  } fix;
  unsigned int num_sats; // Number visible
};


// Initialize nmea struct
void nmea_init(struct nmea* nmea);

// Parse nul-terminated NMEA message
int nmea_parse_msg(struct nmea* nmea, char* msg);

bool nmea_fix_valid(struct nmea* nmea);

void nmea_fix_age(struct nmea* nmea, struct timeval* age);