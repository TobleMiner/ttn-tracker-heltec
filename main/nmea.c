#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/time.h>

#include "nmea.h"
#include "util.h"
#include "strutil.h"

#define FIX_TIMEOUT 30

static bool check_csum(char* msg) {
  uint8_t csum;
  size_t len = strlen(msg);
  // Leading '$' + csum delimiter '*' and csum 'XX'
  if(len < 4) {
    return false;
  }
  // Leading '$' is irrelevant to checksum calculation
  msg++;
  len--;

  // Neither the '*' delimiter nor the checksum itsef 
  // are part of the checksum
  len -= 3;

  // Extract csum at end of mesage
  csum = hex_to_byte(msg + len);

  while(len-- > 1) {
    csum ^= (uint8_t)*msg++;
  }

  // csum should be 0
  return !csum;
}

static char* skip_delim(char* ptr) {
  char* pos = strchr(ptr, ',');
  if(!pos) {
    return NULL;
  }
  // pos can never end up on the terminating nul byte
  // thus pos + 1 is still inside string
  return pos + 1;
}

static int parse_coordinate(char** str, struct coordinate* coord, bool is_lng) {
  struct coordinate val;
  unsigned int deg;
  // int, not off_t because sscanf demands it
  int offset = 0;
  int parsed_args = sscanf(*str, is_lng ? "%3u%lf,%c,%n" : "%2u%lf,%c,%n", &deg, &val.deg, &val.dir, &offset);
  if(parsed_args == 3) {
    // Full expected format
    *str += offset;
  } else {
    // Empty or not parseable. Skip
    char* tmp = *str;
    tmp = skip_delim(tmp);
    if(!tmp) {
      return -EINVAL;
    }
    tmp = skip_delim(tmp);
    if(!tmp) {
      return -EINVAL;
    }
    *str = tmp;
    return 1;
  }
  coord->deg = deg + val.deg / 60.0;
  coord->dir = val.dir;
  return 0;
}

static int parse_coordinate_lat(char** str, struct coordinate* coord) {
  return parse_coordinate(str, coord, false);
}

static int parse_coordinate_lng(char** str, struct coordinate* coord) {
  return parse_coordinate(str, coord, true);
}

static int nmea_parse_gpgsv(struct nmea* nmea, char* msg) {
  return sscanf(msg, "$GPGSV,%*u,%*u,%u", &nmea->num_sats) == 1 ? NMEA_OK : NMEA_SYNTAX_ERROR;
}

#define FIX_VALID(x) ((x) > 0)

static int nmea_parse_gpgga(struct nmea* nmea, char* msg) {
  struct coordinate lat, lng;
  unsigned int fix, num_sats;
  double hdop, alt_msl;
  int num_matched;

  if(!((msg = skip_delim(msg)) && (msg = skip_delim(msg)))) {
    return NMEA_SYNTAX_ERROR;
  }

  if(parse_coordinate_lat(&msg, &lat) < 0) {
    return NMEA_SYNTAX_ERROR;
  }
  if(parse_coordinate_lng(&msg, &lng) < 0) {
    return NMEA_SYNTAX_ERROR;
  }
    
  num_matched = sscanf(msg, "%u,%u,%lf,%lf", &fix, &num_sats, &hdop, &alt_msl);
  // At least the parameter fix quality must be specified
  if(num_matched < 1) {
    return NMEA_SYNTAX_ERROR;
  }

  if(FIX_VALID(fix)) {
    nmea->fix.quality = fix;
    nmea->fix.lat = lat;
    nmea->fix.lng = lng;
    nmea->fix.num_sats = num_sats;
    nmea->fix.hdop = hdop;
    nmea->fix.alt_msl = alt_msl;
    sys_get_time(&nmea->fix.time);
  }

  return NMEA_OK;
}

static int nmea_parse_gpgsa(struct nmea* nmea, char* msg) {
  return sscanf(msg, "$GPGSA,%*c,%u", &nmea->nav.fix) == 1 ? NMEA_OK : NMEA_SYNTAX_ERROR;
}

void nmea_init(struct nmea* nmea) {
  memset(nmea, 0, sizeof(*nmea));
}

int nmea_parse_msg(struct nmea* nmea, char* msg) {
  if(!check_csum(msg)) {
    return NMEA_CSUM_INVALID;
  }

  if(startswith(msg, "$GPGSV")) {
    return nmea_parse_gpgsv(nmea, msg);
  }

  if(startswith(msg, "$GPGGA")) {
    return nmea_parse_gpgga(nmea, msg);
  }

  if(startswith(msg, "$GPGSA")) {
    return nmea_parse_gpgsa(nmea, msg);
  }

  return NMEA_UNKNOWN_MESSAGE;
}

bool nmea_fix_valid(struct nmea* nmea) {
  struct timeval diff = {
    .tv_sec = FIX_TIMEOUT,
    .tv_usec = 0
  };
  struct timeval now;
  struct timeval timeout;
  if(!FIX_VALID(nmea->fix.quality)) {
    return false;
  }
  sys_get_time(&now);
  timeradd(&nmea->fix.time, &diff, &timeout);
  return timercmp(&now, &timeout, <);
}

void nmea_fix_age(struct nmea* nmea, struct timeval* age) {
  struct timeval now;
  sys_get_time(&now);
  timersub(&now, &nmea->fix.time, age);
}

bool nmea_fix_2d(struct nmea* nmea) {
  return nmea_fix_valid(nmea) && nmea->nav.fix == 2 &&
    (nmea->fix.quality == 1 || nmea->fix.quality == 2);
}
bool nmea_fix_3d(struct nmea* nmea) {
  return nmea_fix_valid(nmea) && nmea->nav.fix == 3 &&
    (nmea->fix.quality == 1 || nmea->fix.quality == 2);
}