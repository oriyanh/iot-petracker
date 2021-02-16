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
static bool isOpen = false;
static unsigned char GPS_RX_BUFFER[GPS_BUFFER_SIZE] = {0};
static int gpsHandle = -1;


static int parseGPSData(GPS_LOCATION_INFO *location, char *buf, uint32_t len);

/**
 * @brief Initialises the serial connection.
 * @param port - the port to connected to. e.g: /dev/ttyUSB0, /dev/ttyS1 for Linux and COM8, COM10, COM53 for Windows.
 * @param baud - the baud rate of the communication. For example: 9600, 115200
 * @return 0 if succeeded in opening the port and -1 otherwise.
 */
int GPSInit()
{
    logDebug("GPSInit");
    if (isOpen)
    {
        logWarn("GPS already initialized");
        return 0;
    }

    logDebug("GPS Serial Init");
    int result = SerialInit(GPS_PORT, GPS_BAUDRATE, &gpsHandle);
    if (result < 0)
    {
        logError("GPSInit fail");
        return result;
    }
    logDebug("GPSInit success");
    isOpen = true;
    return 0;
}

uint32_t GPSGetReadRaw(unsigned char *buf, unsigned int max_len)
{
    if (!isOpen)
    {
        logWarn("GPS is not initialized");
        return 0;
    }
    SerialFlushInputBuff(&gpsHandle);
    int result = SerialRecv(buf, max_len, SHORT_TIMEOUT, &gpsHandle);
    return result;
}

bool GPSGetFixInformation(GPS_LOCATION_INFO *location)
{
    uint32_t bytesRecv = GPSGetReadRaw(GPS_RX_BUFFER, GPS_BUFFER_SIZE);
    if (bytesRecv <= 0)
    {
        logError("Failed to read GPS data");
        return false;
    }

    GPS_RX_BUFFER[bytesRecv] = '\0';
    int res = parseGPSData(location, (char*)GPS_RX_BUFFER, bytesRecv);
    if (res < 0)
    {
        logError("GPSGetFixInformation error");
        return false;
    }


    return true;

}

void GPSDisable()
{
    if (isOpen)
    {
        SerialDisable(&gpsHandle);
        isOpen = false;
    }
}

#define GPGGA "$GPGGA"
#define GPGSA "$GPGSA"
#define GPRMC "$GPRMC"
#define NMEA_MSGID_LEN 5
#define MAX_NMEA_MSG_LEN 83 // NMEA 083 protocol dictates message length is maximum 79 chars + 3 for delimiters, so we add 1 more for \0

bool validateNMEAChecksum(const char* message, size_t len);

int parseGPGGAMessage(GPS_LOCATION_INFO *location, const char *message, uint32_t len);

int parseGPGSAMessage(GPS_LOCATION_INFO *location, char *message, uint32_t len);

int parseGPRMCMessage(GPS_LOCATION_INFO *location, char *message, uint32_t len);

int parseGPSData(GPS_LOCATION_INFO *location, char *buf, uint32_t len)
{

    memset(location, 0, sizeof(GPS_LOCATION_INFO));
    char *startOp = NULL, *endOp = NULL;
    int result = 0;
    startOp = buf;
    uint32_t bytesRemaining = len;
    size_t messageLen = 0;
    uint32_t i = 0;
    for (; i < (len-1) && buf[i] != '$'; i++) {}
    if (buf[i] != '$')
    {
        logWarn("Message start '$' not detected");
        return -1;
    }
    startOp = buf + i;
    i++;
    for (; i < len; i++)
    {
        if (buf[i] != '$')
        {
            continue;
        }
        endOp = buf + i;
        messageLen = endOp - startOp;
        logDebug("Parsing message: '%.*s'", messageLen, startOp);
        if (!validateNMEAChecksum(startOp, messageLen))
        {
            logWarn("NMEA message failed checksum, skipping");
        }
        else if (strncmp(startOp, GPGGA, NMEA_MSGID_LEN) == 0)
        {
            parseGPGGAMessage(location, startOp, messageLen);
        }
        else if (strncmp(startOp, GPGSA, NMEA_MSGID_LEN) == 0)
        {
            parseGPGSAMessage(location, startOp, messageLen);
        }
        else if (strncmp(startOp, GPRMC, NMEA_MSGID_LEN) == 0)
        {
            parseGPRMCMessage(location, startOp, messageLen);
        }
        bytesRemaining -= messageLen;
        logDebug("bytesRemaining=%zu", bytesRemaining);
        if ((i+1 < len))
        {
            startOp = endOp;
        }
    }
 
    return result;
}

bool validateNMEAChecksum(const char* message, size_t len)
{
        char *checksumStr = NULL;
        checksumStr = strchr((const char*)message, '*');
        if (checksumStr == NULL)
        {
            logWarn("Invalid NMEA message, no checksum field");
            return false;
        }

        int expectedChecksum = 0;
        expectedChecksum = (int)strtol((const char*)checksumStr+1, NULL, 16);

        int actualChecksum = 0;
        for (size_t i=0; i < len; i++)
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

        logDebug("Expected checksum: 0x%X, actual checksum: 0x%X", expectedChecksum, actualChecksum);
        return actualChecksum == expectedChecksum;
}


int32_t parseNMEACoordinate(const char* coord, size_t len)
{
    char *decimelPt = NULL;
    decimelPt = strchr(coord, '.');
    if (decimelPt == NULL)
    {
        logWarn("Failed parseNMEACoordinate, illegal format");
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
    memset(MESSAGE_BUFFER, 0, MAX_NMEA_MSG_LEN);
    strncpy(MESSAGE_BUFFER, message, len);
    MESSAGE_BUFFER[len] = '\0';

    char* startOp = NULL;
    int result = 0;
    static const char* SEPARATORS = ",";
    startOp = strtok(MESSAGE_BUFFER, SEPARATORS);
    int fieldNum = 0;
    char *decimelPt = NULL;
    while (startOp != NULL)
    {
        logDebug("field #%d=%s", fieldNum, startOp);
        switch (fieldNum)
        {
            case UTC_LOC:
                decimelPt = strchr(startOp, '.');
                if (decimelPt == NULL)
                {
                    logWarn("Illegal UTC format");
                    continue;
                }
                memset(location->fixtime, 0, 10);
                // UTC Time in format ‘hhmmss.sss'
                strncpy(location->fixtime, startOp, 6);
                decimelPt++;
                strncat(location->fixtime, decimelPt, 3);
                break;
            case LAT_LOC:
                // Latitude in format ‘ddmm.mmmm’ (degree and minutes)
                location->latitude = parseNMEACoordinate(startOp, strlen(startOp));
                break;
            case LON_LOC:
                // longitude in format ‘dddmm.mmmm’ (degree and minutes)
                location->longitude = parseNMEACoordinate(startOp, strlen(startOp));
                break;
            case ALTITUDE_LOC:
                location->altitude = (int32_t)(strtold(startOp, NULL)*100.0);
                break;
            case LAT_DIR_LOC:
                if (startOp[0] == 'S')
                {
                    location->latitude = -location->latitude;
                }
                break;
            case LON_DIR_LOC:
                if (startOp[0] == 'W')
                {
                    location->longitude = -location->longitude;
                }
                break;
            case GPS_QUALITY_LOC:
                location->valid_fix = (uint8_t) strtoul(startOp, NULL, 10);
                break;
            case NUM_SATELLITES_LOC:
                location->num_sats = (uint8_t) strtoul(startOp, NULL, 10);
                break;
            case HDOP_LOC:
                location->hdop = (uint8_t)(strtold(startOp, NULL)*5.0);
                break;
            default:
                break;
        }
        ++fieldNum;
        startOp = strtok(NULL, SEPARATORS);
        decimelPt = NULL;
    }

    logInfo("GPS: LAT=%d, LONG=%d, ALT=%d, HDOP=%u, FIX=%u, NSATS=%u, UTC=%s",
    location->latitude, location->longitude, location->altitude,\
    location->hdop, location->valid_fix, location->num_sats, location->fixtime);
    return result;
}

int parseGPGSAMessage(GPS_LOCATION_INFO *location, char *message, uint32_t len)
{
    return 0;
}

int parseGPRMCMessage(GPS_LOCATION_INFO *location, char *message, uint32_t len)
{
    return 0;
}

