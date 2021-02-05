/*
 * config.h
 *
 *  Created on: 31 בדצמ× 2020
 *      Author: Maya lulko
 */


#ifndef SRC_CONFIG_H_
#define SRC_CONFIG_H_

//#define NO_LOGGER

#define BUFFER_SIZE 4096

#define SHORT_TIMEOUT 3000
#define MEDIUM_TIMEOUT 4000
#define LONG_TIMEOUT 5000

// According to Wikipedia, ICCID should be at most 22 digits
#define ICCID_NUM_DIGITS 23

#define MODEM_PORT ""
#define BAUDRATE 115200
#define MAX_NUMBER_RETRIES 30
#define SERIAL_ENDL "\r\n"
#define SYS_ENDL "\n"


#define MQTT_DASHBOARD "35.156.182.231"
#define TOPIC_NAME "huji_iot_class/2020_2021"
#define MQTT_PORT 1883
#define MQTT_CMD_TIMEOUT_MS 1500
#define DEFAULT_KEEP_ALIVE_SEC 60
#define PAYLOAD_SIZE 512u
#define PAYLOAD_FORMAT "{" \
                "\"ICCID\":\"%s\", " \
                "\"OperatorName\":\"%s\", " \
                "\"OperatorCode\":\"%d\", " \
                "\"Technology\":\"%s\", " \
                "\"Signal\":\"%d dBm\", " \
				"\"Clicks\":%ld"       \
                "}"

#endif /* SRC_CONFIG_H_ */

