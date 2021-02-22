#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "config.h"
#include "serial_io.h"
#include "gps.h"
#include "logger.h"

#define GPS_BUFFER_SIZE 2048
#define GPGGA "$GPGGA"
#define NMEA_MSGID_LEN 5
#define MAX_NMEA_MSG_LEN 83 // NMEA 083 protocol dictates message length is maximum 79 chars + 3 for delimiters, so we add 1 more for \0

static bool isOpen = false;
static unsigned char GPS_RX_BUFFER[GPS_BUFFER_SIZE] = {0};
static int gpsHandle = -1;

/**
 * @brief Parses NMEA messages grabbed from GPS
 * @param GPS_LOCATION_INFO pre-allocated pointer that will be populated by this function
 * @param buf buffer containing raw GPS data, data to parse
 * @param len Length of buffer
 * @return 0 if succeeded in parsing data, -1 otherwise
 */
static int parseGPSData(GPS_LOCATION_INFO *location, char *buf, uint32_t len);

/**
 * @brief Parses single GPGGA NMEA message
 * @param GPS_LOCATION_INFO pre-allocated pointer that will be populated by this function
 * @param message message to parse, includes prefix $GPGGA, may include whitespace
 * @param len message length
 * @return 0 if succeeded in parsing message, -1 otherwise
 */
static int parseGPGGAMessage(GPS_LOCATION_INFO *location, const char *message, uint32_t len);

/**
 * @brief Parses single NMEA coordinate from string
 * @param coord coordinate to parse, in format [d]ddmm.mmmm where d is Degrees and m is Minutes, and [d] is an optional digit
 * @param len length of coord string
 * @return parsed coordinate as an int32, equal to the decimel representation of `coord` x 10^7.
 */
static int32_t parseNMEACoordinate(const char* coord, size_t len);

/**
 * @brief Calculates checksum of NMEA message
 * @param message message to validate, includes suffix *XX where XX is the checsum of the message in base16
 * @param len message length
 * @return true if checksum is valid, false otherwise
 */
static bool validateNMEAChecksum(const char* message, size_t len);

int GPSInit()
{
  logInfo("GPSInit");
  if (isOpen)
  {
    logWarn("GPS already initialized");
    return 0;
  }

  logInfo("GPS SerialInit");
  int result = SerialInit(GPS_PORT, GPS_BAUDRATE, &gpsHandle);
  if (result < 0)
  {
    logError("GPSInit failed SerialInit");
    return result;
  }

  isOpen = true;
  return 0;
}

int GPSGetReadRaw(unsigned char *buf, unsigned int max_len)
{
  logInfo("GPSGetReadRaw");
  if (!isOpen)
  {
    logError("GPSGetReadRaw fail, GPS not initialized");
    return -1;
  }

  if (buf == NULL)
  {
    logError("GPSGetReadRaw fail, buf is NULL");
    return -1;
  }

  SerialFlushInputBuff(&gpsHandle);
  int result = SerialRecv(buf, max_len, MEDIUM_TIMEOUT, &gpsHandle);
  if (result < 0)
  {
    logError("GPSGetReadRaw SerialRecv fail");
    return -1;
  }

  return result;
}

bool GPSGetFixInformation(GPS_LOCATION_INFO *location)
{
  logInfo("GPSGetFixInformation");
  int bytesRecv = GPSGetReadRaw(GPS_RX_BUFFER, GPS_BUFFER_SIZE);
  if (bytesRecv <= 0)
  {
    logError("GPSGetFixInformation read fail");
    return false;
  }

  GPS_RX_BUFFER[bytesRecv] = '\0';
  int res = parseGPSData(location, (char*)GPS_RX_BUFFER, bytesRecv);
  if (res < 0)
  {
    logError("GPSGetFixInformation parsing error");
    return false;
  }

  return true;
}

void GPSDisable()
{
  logInfo("GPSDisable");
  if (isOpen)
  {
    SerialDisable(&gpsHandle);
    gpsHandle = -1;
    isOpen = false;
  }
}

int parseGPSData(GPS_LOCATION_INFO *location, char *buf, uint32_t len)
{
  memset(location, 0, sizeof(GPS_LOCATION_INFO));
  char *messageStart = NULL, *messageEnd = NULL;
  messageStart = buf;

  uint32_t bytesRemaining = len;
  size_t messageLen = 0;
  uint32_t i = 0;
  // Find first message that starts with $ prefix, exit if not detected
  for (; i < (len-1) && buf[i] != '$'; i++) {}
  if (buf[i] != '$')
  {
    logError("parseGPSData fail, incomplete message");
    return -1;
  }

  messageStart = buf + i;
  ++i;
  // todo handle remainingBytes correctly,  we are not parsing the last chunk of data
  for (; i < len; i++)
  {
    // Find start of next message
    if (buf[i] != '$')
      // if (buf[i] != '$' && buf[i] != '\n' && buf[i] != '\r')
    {
      continue;
    }

    messageEnd = buf + i;
    messageLen = messageEnd - messageStart;
    logDebug("Parsing message: '%.*s'", messageLen, messageStart);
    if (!validateNMEAChecksum(messageStart, messageLen))
    {
      logWarn("NMEA message failed checksum, skipping");
    }

    else if (strncmp(messageStart, GPGGA, NMEA_MSGID_LEN) == 0)
    {
      parseGPGGAMessage(location, messageStart, messageLen);
    }

    bytesRemaining -= messageLen;
    logDebug("bytesRemaining=%zu", bytesRemaining);
    if ((i+1 < len))
    {
      messageStart = messageEnd;
    }
  }

  return 0;
}

bool validateNMEAChecksum(const char* message, size_t len)
{
  if (message == NULL)
  {
    logError("validateNMEAChecksum fail, NULL message");
    return false;
  }

  char *checksumStr = NULL;
  checksumStr = strchr((const char*)message, '*');
  if (checksumStr == NULL)
  {
    logError("validateNMEAChecksum fail, no checksum");
    return false;
  }

  int expectedChecksum = 0;
  expectedChecksum = (int)strtol((const char*)checksumStr+1, NULL, 16);

  int actualChecksum = 0;
  for (size_t i=0; i < (checksumStr - message); i++)
  {
    if (message[i] == '$')
    {
      continue;
    }
    else if (message[i] == '*')
    {
      break;
    }
    actualChecksum ^= message[i];
  }

  if (actualChecksum != expectedChecksum)
  {
    logError("Checksum fail, expected=0x%X, actual=0x%X", expectedChecksum, actualChecksum);
  }

  return actualChecksum == expectedChecksum;
}

int32_t parseNMEACoordinate(const char* coord, size_t len)
{
  if (coord == NULL)
  {
    logWarn("Failed parseNMEACoordinate, NULL coord");
    return 0;
  }

  char *decimelPt = NULL;
  decimelPt = strchr(coord, '.');
  if (decimelPt == NULL)
  {
    logWarn("Failed parseNMEACoordinate, illegal format");
    return 0;
  }
  char *degreesPt = decimelPt - 2;

  int32_t degrees = 0;
  int32_t minutes = 0;
  size_t loc = 0;
  for (loc=0; loc < (size_t)(degreesPt-coord); loc++)
  {
    degrees *= 10;
    degrees += coord[loc] - '0';
  }

  for (loc = degreesPt-coord; loc < len; loc++)
  {
    if ((coord + loc) == decimelPt)
    {
      loc++;
    }

    minutes *= 10;
    degrees *= 10;
    minutes += coord[loc] - '0';
  }
  minutes *= 1000;
  degrees *= 10;
  logDebug("degrees=%d, minutes=%d, parsedCoord=%d",degrees, minutes, degrees + minutes/60);
  return degrees + minutes/60;
}

#define UTC_LOC 1
#define LAT_LOC 2
#define LAT_DIR_LOC 3
#define LON_LOC 4
#define LON_DIR_LOC 5
#define GPS_QUALITY_LOC 6
#define NUM_SATELLITES_LOC 7
#define HDOP_LOC 8
#define ALTITUDE_LOC 9

int parseGPGGAMessage(GPS_LOCATION_INFO *location, const char *message, uint32_t len)
{
  static char MESSAGE_BUFFER[MAX_NMEA_MSG_LEN] = {0};
  logInfo("parseGPGGAMessage");
  if (location == NULL)
  {
    logError("parseGPGGAMessage fail, NULL location");
  }

  if (message == NULL)
  {
    logError("parseGPGGAMessage fail, NULL message");
  }

  if (len > (MAX_NMEA_MSG_LEN-1))
  {
    logWarn("parseGPGGAMessage len=%u>%u=MAX_NMEA_MSG_LEN, truncating message to fit", len, MAX_NMEA_MSG_LEN-1);
    len = MAX_NMEA_MSG_LEN;
  }
  memset(MESSAGE_BUFFER, 0, MAX_NMEA_MSG_LEN);
  strncpy(MESSAGE_BUFFER, message, len);
  MESSAGE_BUFFER[len] = '\0';

  char* messageStart = NULL;
  int result = 0;
  static const char* SEPARATORS = ",";
  messageStart = strtok(MESSAGE_BUFFER, SEPARATORS);
  int fieldNum = 0;
  char *decimelPt = NULL;
  while (messageStart != NULL)
  {
    logDebug("field #%d=%s", fieldNum, messageStart);
    switch (fieldNum)
    {
      case UTC_LOC:
        decimelPt = strchr(messageStart, '.');
        if (decimelPt == NULL)
        {
          logWarn("GPGGA Illegal UTC format");
          continue;
        }
        memset(location->fixtime, 0, 10);
        // UTC Time is in format 'hhmmss.sss'
        strncpy(location->fixtime, messageStart, 6);
        decimelPt++;
        strncat(location->fixtime, decimelPt, 3);
        break;

      case LAT_LOC:
        // Latitude in format 'ddmm.mmmm' (degree and minutes)
        location->latitude = parseNMEACoordinate(messageStart, strlen(messageStart));
        break;

      case LON_LOC:
        // longitude in format 'ddmm.mmmm' (degree and minutes)
        location->longitude = parseNMEACoordinate(messageStart, strlen(messageStart));
        break;

      case ALTITUDE_LOC:
        location->altitude = 0;
        // ALTITUDE is in format 'm.cc' where m is altitude in meters (variable num of digits), and cc is centimeters (2 digits)
        location->altitude = (int32_t)(strtol(messageStart, NULL, 10)*100);
        decimelPt = strchr(messageStart, '.');
        if (decimelPt == NULL)
        {
          logWarn("GPGGA ALTITUDE missing decimal point");
          break;
        }
        decimelPt++;
        location->altitude += (decimelPt[0] - '0')*10;
        location->altitude += decimelPt[1] - '0';
        break;

      case LAT_DIR_LOC:
        if (messageStart[0] == 'S')
        {
          location->latitude = -location->latitude;
        }
        break;

      case LON_DIR_LOC:
        if (messageStart[0] == 'W')
        {
          location->longitude = -location->longitude;
        }
        break;
      case GPS_QUALITY_LOC:
        location->valid_fix = (uint8_t) strtoul(messageStart, NULL, 10);
        break;

      case NUM_SATELLITES_LOC:
        location->num_sats = (uint8_t) strtoul(messageStart, NULL, 10);
        break;

      case HDOP_LOC:
        location->hdop = (uint8_t)(strtold(messageStart, NULL)*5.0);
        break;

      default:
        break;
    }
    ++fieldNum;
    messageStart = strtok(NULL, SEPARATORS);
    decimelPt = NULL;
  }

  logInfo("GPS: LAT=%d, LONG=%d, ALT=%d, HDOP=%u, FIX=%u, NSATS=%u, UTC=%s",
          location->latitude, location->longitude, location->altitude,\
          location->hdop, location->valid_fix, location->num_sats, location->fixtime);
  return result;
}
