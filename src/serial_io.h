#ifndef SERIAL_IO_H
#define SERIAL_IO_H

/**
 * @brief Initialises the serial connection.
 * @param port - MODEM_PORT or GPS_PORT
 * @param baud - the baud rate of the communication. For example: 9600, 115200
 * @param handle pointer to handle used to identify serial port that was opened
 * @return 0 if succeeded in opening the port and -1 otherwise.
 */
int SerialInit(SERIAL_PORT port, unsigned int baud, int *handle);

/**
 * @brief Receives data from serial connection.
 * @param buf - the buffer that receives the input.
 * @param max_len - maximum bytes to read into buf (buf must be equal or greater than max_len).
 * @param timeout_ms - read operation timeout milliseconds.
 * @param handle pointer to handle used to identify serial port that was opened
 * @return amount of bytes read into buf, -1 on error.
 */
int SerialRecv(unsigned char *buf, unsigned int max_len, unsigned int timeout_ms, int *handle);

/**
 * @brief Sends data through the serial connection.
 * @param buf - the buffer that contains the data to send
 * @param size - number of bytes to send
 * @param handle pointer to handle used to identify serial port that was opened
 * @return amount of bytes written into buf, -1 on error
 */
int SerialSend(const unsigned char *buf, unsigned int size, int *handle);

/**
 * @brief Empties the input buffer.
 * @param handle pointer to handle used to identify serial port that was opened
 */
void SerialFlushInputBuff(int *handle);

/**
 * @brief Disable the serial connection.
 * @param handle pointer to handle used to identify serial port that was opened
 * @return 0 if succeeded in closing the port and -1 otherwise.
 */
int SerialDisable(int *handle);

#endif // SERIAL_IO_H
