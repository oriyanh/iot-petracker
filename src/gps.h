#ifndef IOT_GPS_H
#define IOT_GPS_H

#include <stdint.h>
#include <stdbool.h>
typedef struct __attribute__ ((__packed__)) _GPS_LOCATION_INFO
{
	int32_t latitude; // scaled up x 10^7
	int32_t longitude; // scaled up x 10^7
	int32_t altitude; // scaled up x 10^2
	uint8_t hdop; // scaled x 5
	uint8_t valid_fix : 2;
	uint8_t reserved1 : 1;
	uint8_t num_sats : 5; // scaled x 1
	char fixtime[10]; // String rep. of UTC time
} GPS_LOCATION_INFO;


/** nitialize whatever is needed to start working with the GPS (e.g. the serial port)
 * @param host is the destination address as DNS: en8wtnrvtnkt5.x.pipedream.net, or as IPv4: 35.169.0.97.
 * @param port the communication endpoint, as string, e.g.: “80”.
 * @return 0 on success, -1 on failure.
 */
int GPSInit();

/**
 * Receives a pointer to a GPS_LOCATION_INFO struct and fills it with up to date information from the GPS.
 * @param payload Buffer to write to the socket
 * @param len Length of `payload` in bytes
 * @return the number of bytes written on success, -1 on failure.
 */
bool GPSGetFixInformation(GPS_LOCATION_INFO *location);

/**
 * Fills bufwith up to maxlencharacters from the GPS’s output. Returns the number of chars written to buf.
 * @param buf Buffer to populate with response. Assumes enough memory is allocated.
 * @param max_len Maximum number of bytes to read from socket
 * @return the number of bytes read on success, -1 on failure.
 */
uint32_t GPSGetReadRaw(unsigned char *buf, unsigned int max_len);

/**
 * Deallocate / close whatever resources that were allocated on GPSInit()
 * @return 0 on success, -1 on failure.
 */
void GPSDisable();

#endif //IOT_GPS_H
