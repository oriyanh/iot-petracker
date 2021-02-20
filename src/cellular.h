#ifndef IOT_CELLULAR_H
#define IOT_CELLULAR_H

typedef struct __OPERATOR_INFO
{
  char operatorName[10];  // Long name. See <format> under +COPS.
  int operatorCode;  // Short code. See <format> under +COPS.
  char accessTechnology[4];  // "2G" or "3G"
} OPERATOR_INFO;

typedef struct __SIGNAL_INFO
{
  int signal_power; // In 2G: dBm. In 3G: RSCP. See ^SMONI responses.
  int EC_n0; // In 3G only. See ^SMONI responses.
  char accessTechnology[4]; // "2G" or "3G"
} SIGNAL_INFO;

typedef struct __MODEM_METADATA
{
  char ccid[23];
  OPERATOR_INFO op_info;
  int csq;
} MODEM_METADATA;

/**
 * Initialize whatever is needed to start working with the cellular modem (e.g. the serial port). Returns 0 on success, and -1 on failure.
 * @return
 */
int CellularInit(void);

/**
 * Soft reboot of cellular modem + verify modem has initialized correctly
 */
int CellularReboot(void);

/**
 * Deallocate / close whatever resources CellularInit() allocated.
 */
int CellularDisable(void);

/**
 * Checks if the modem is responding to AT commands. Return 0 if it does, returns -1 otherwise.
 * @return
 */
int CellularCheckModem(void);

/**
 * Returns -1 if the modem did not respond or respond with an error.
 * Returns 0 if the command was successful and the registration status was obtained from the modem.
 * In that case, the statusparameter will be populated with the numeric value of the <regStatus>field of the ג€�+CREGג€� AT command.
 * @param status
 * @return
 */
int CellularGetRegistrationStatus(int *status);

/**
 * orces the modem to search for available operators (see ג€�+COPS=?ג€� command).
 * Returns -1 if an error occurred or no operators found. Returns 0 and populates opList and opsFound
 * if the command succeeded. opList is a pointer to the first item of an array of type OPERATOR_INFO,
 * which is allocated by the caller of this function. The array contains a total of maxops items.
 * numOpsFoundis allocated by the caller and set by the function. numOpsFound will contain
 * the number of operators found and populated into the opList.
 * @param opList
 * @param maxops
 * @param numOpsFound
 * @return
 */
int CellularGetOperators(OPERATOR_INFO *opList, int maxops, int *numOpsFound);

/**
 * Forces the modem to register/deregister with a network.
 * Returns 0 if the command was successful, returns -1 otherwise.
 * If mode=0, sets the modem to automatically register with an operator (ignores the operatorCode parameter).
 * If mode=1, forces the modem to work with a specific operator, given in operatorCode.
 * If mode=2, deregisters from the network (ignores the operatorCode parameter).
 * See the “+COPS=<mode>,...” command for more details.
 * @param mode
 * @param operatorCode
 * @return
 */
int CellularSetOperator(int mode, OPERATOR_INFO* op);

/**
 * Initialize an internet connection profile (AT^SICS) with inactTO=inact_time_sec and
 * conType=GPRS0 and apn="postm2m.lu". Return 0 on success, and -1 on failure.
 * @param inact_time_sec
 * @return 0 on success, -1 on failure
 */
int CellularSetupInternetConnectionProfile(int inact_time_sec);

/**
 * Initialize an internal service profile (AT^SISS) with keepintvl=keepintvl_sec (the timer) and
 * SrvType=Socket, and conId=<CellularSetupInternetConnectionProfile_id>
 * (if CellularSetupInternetConnectionProfile is already initialized. Return error, -1, otherwise) and
 * Address=socktcp://IP:port;etx;time=keepintvl_sec. Return 0 on success, and -1 on failure.
 * @param IP
 * @param port
 * @param keepintvl_sec
 * @return 0 on success, -1 on failure
 */
int CellularSetupInternetServiceSetupProfile(const char *IP, unsigned int port, int keepintvl_sec);

/**
 * Connects to the socket (establishes TCP connection to the pre-defined host and port).
 * @return 0 on success, -1 on failure
 */
int CellularConnect(void);

/**
 * Closes the established connection.
 * @return 0 on success, -1 on failure
 */
int CellularClose();

/**
 * Writes len bytes from payload buffer to the established connection.
 * @param payload
 * @param len
 * @return number of bytes on success, -1 on failure
 */
int CellularWrite(const unsigned char *payload, unsigned int len);

/**
 * Reads up to max_len bytes from the established connection to the provided buf buffer, for up to timeout_ms (doesnג€™t block
 * longer than that, even if not all max_len bytes were received).
 * @param buf
 * @param max_len
 * @param timeout_ms
 * @return number of bytes on success, -1 on failure
 */
int CellularRead(unsigned char *buf, unsigned int max_len, unsigned int timeout_ms);

/**
 * Populates the metadata struct with ccid, csq, registered network information.
 */
void GetModemMetadata(MODEM_METADATA *metaData);

#endif //IOT_CELLULAR_H
