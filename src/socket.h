#ifndef IOT_SOCKET_H
#define IOT_SOCKET_H
#include "wolfmqtt/mqtt_types.h"

/**
 * Initializes a TCP socket.
 * @param host is the destination address as IPv4: 35.169.0.97.
 * @param port the communication endpoint
 * @return 0 on success, -1 on failure.
 */
int SocketInit(const char *host, word16 port);

/**
 * Connects to the socket (establishes TCP connection to the pre-defined host and port, as defined in `SocketInit()`).
 * @return 0 on success, -1 on failure.
 */
int SocketConnect(void);

/**
 * Writes `len` bytes from `payload` buffer to the established connection
 * @param payload Buffer to write to the socket
 * @param len Length of `payload` in bytes
 * @return the number of bytes written on success, -1 on failure.
 */
int SocketWrite(const byte *payload, unsigned int len);

/**
 * Reads up to `max_len` bytes from the established connection to the provided `buf` buffer,
 * for up to `timeout_ms` (doesn’t block longer than that, even if not all `max_len` bytes were received).
 * @param buf Buffer to populate with response. Assumes enough memory is allocated.
 * @param max_len Maximum number of bytes to read from socket
 * @param timeout_ms Timeout in ms
 * @return the number of bytes read on success, -1 on failure.
 */
int SocketRead(byte *buf, unsigned int max_len, unsigned int timeout_ms);

/**
 * Closes the established connection/socket
 * @return 0 on success, -1 on failure.
 */
int SocketClose();

/**
 * Closes the socket and resets all configurations
 * @return 0 on success, -1 on failure.
 */
int SocketDisable();

#endif //IOT_SOCKET_H
