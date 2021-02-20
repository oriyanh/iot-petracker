#ifndef IOT_GPS_H
#define IOT_GPS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct __attribute__ ((__packed__)) _GPS_LOCATION_INFO
{
  int32_t latitude; // decimel value scaled up x 10^7
  int32_t longitude; // decimel value scaled up x 10^7
  int32_t altitude; // in meters, scaled up x 10^2
  uint8_t hdop; // scaled x 5
  uint8_t valid_fix : 2;
  uint8_t reserved1 : 1;
  uint8_t num_sats : 5;
  char fixtime[10]; // String rep. of UTC time, null terminated, format hhmmssMMM where h is hour, m is minute, s is second, M is millisecond
} GPS_LOCATION_INFO;


/** Initialize whatever is needed to start working with the GPS (e.g. the serial port)
 * @return 0 on success, -1 on failure.
 */
int GPSInit();

/**
 * Receives a pointer to a GPS_LOCATION_INFO struct and fills it with up to date information from the GPS.
 * @param GPS_LOCATION_INFO pre-allocated pointer that will be populated by this function
 * @return true if successfully acquired location info, false otherwise
 */
bool GPSGetFixInformation(GPS_LOCATION_INFO *location);

/**
 * Fills buf with up to max_len characters from the GPS’s output. Returns the number of chars written to buf.
 * @param buf Buffer to populate with response. Assumes enough memory is allocated.
 * @param max_len Maximum number of bytes to read from GPS
 * @return the number of bytes read on success, -1 on failure.
 */
int GPSGetReadRaw(unsigned char *buf, unsigned int max_len);

/**
 * Deallocate / close whatever resources that were allocated on GPSInit()
 * @return 0 on success, -1 on failure.
 */
void GPSDisable();

#endif //IOT_GPS_H
