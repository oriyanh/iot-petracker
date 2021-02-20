#ifndef SRC_CONFIG_H_
#define SRC_CONFIG_H_

//#define NO_LOGGER

#define BUFFER_SIZE 4096

#define SHORT_TIMEOUT 3000
#define MEDIUM_TIMEOUT 4000
#define LONG_TIMEOUT 5000

// According to Wikipedia, ICCID should be at most 22 digits
#define ICCID_NUM_DIGITS 23

typedef enum _SERIAL_PORT {MODEM_PORT = 0, GPS_PORT} SERIAL_PORT;
#define MODEM_PORT_NAME "CELLULAR_MODEM\0"
#define MODEM_BAUDRATE 115200
#define GPS_PORT_NAME "GPS\0"
#define GPS_BAUDRATE 9600


#define MAX_NUMBER_RETRIES 30
#define SERIAL_ENDL "\r\n"
#define SYS_ENDL "\n"


#define MQTT_DASHBOARD "52.29.249.84"
//#define MQTT_DASHBOARD "35.156.182.231"
//#define MQTT_DASHBOARD "35.158.189.129"

#define LOCATION_TOPIC "petracker/location_raw"
#define DISTRESS_TOPIC "petracker/distress"
#define MQTT_PORT 1883
#define MQTT_CMD_TIMEOUT_MS 5000
#define DEFAULT_KEEP_ALIVE_SEC 120
#define MQTT_PAYLOAD_SIZE 512u

#define DISTRESS_ENABLE "enable"
#define DISTRESS_DISABLE "disable"

#endif /* SRC_CONFIG_H_ */

