#include <stdio.h>

#include "timer_manager.h"
#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "cellular.h"
#include "serial_io.h"
#include "logger.h"

/******* Instantiate data structures, constants, and type definitions ********/
#define _RX_MODEM_BUFFER_SIZE 4096
#define _TX_MODEM_BUFFER_SIZE 100
#define GSM_TECH_INDEX 0
#define UMTS_TECH_INDEX 2

static char CMD_BUFFER[_TX_MODEM_BUFFER_SIZE] = {0};
static char RESULT_BUFFER[_RX_MODEM_BUFFER_SIZE] = {0};

#define CLEAR_RX_BUFFER() memset(RESULT_BUFFER, 0, _RX_MODEM_BUFFER_SIZE)
#define CLEAR_TX_BUFFER() memset(CMD_BUFFER, 0, _TX_MODEM_BUFFER_SIZE)

static const char* TECHNOLOGIES[3] = {"2G", "", "3G"};
static const char* OK_END_MSG = "OK";

enum RETURN_CODE {RET_SUCCESS = 0, RET_ERROR = -1, RET_WARNING = -2};

static OPERATOR_INFO op_info = {0};
static MODEM_METADATA cellularMetaData = { 0 };

/************************ utility functions *****************************/
static void refreshModemMetadata(void);

/**
 * Executes `command` on modem
 * @param command Command to execute
 * @param commandLen Command length
 * @return ERROR if a fatal error has occurred, WARNING if a non fatal error occurred, and number of sent bytes if successful.
 */
static int executeCommand(const char* command, int commandLen) {
    logInfo("Executing command `%s`", command);
    int commandLenUpdated = commandLen + (int)strlen(SERIAL_ENDL);
    // Pre-formatting command
    CLEAR_TX_BUFFER();
    snprintf(CMD_BUFFER, commandLenUpdated + 1, "%s%s%c", command, SERIAL_ENDL, '\0');
    int bytesSent = SerialSend((unsigned char*)CMD_BUFFER, commandLenUpdated+1);
    CMD_BUFFER[0]='\0';

    if (bytesSent < 0) {
        logError("Error occurred while sending command to modem");
        return RET_ERROR;
    }

    logDebug("Successfully sent %d bytes to modem", bytesSent);
    return bytesSent;
}

/**
 * Reads output from modem until encountering `endMsg` in the output buffer, and printing the output to stdout.
 * Waits `timeout` milliseconds between each attempt to read, until `maxNumAttempts` has elapsed.
 *
 * @param buf Input buffer, assumed to be zeroed
 * @param bufsize Size of input buffer
 * @param timeout Timeout in millisecond between attempts
 * @param endMsg string to look for in input buffer, encountering this will cause function to return
 * @param maxNumAttempts Maximum number of attempts to execute command
 * @return ERROR if a fatal error has occurred, WARNING if a non fatal error occurred, and number of sent received if successful.
 */
static int readUntil(char* buf, int bufsize, int timeout, const char* endMsg, int maxNumAttempts){
    int bytesRecv;
    int totalBytesRecv = 0;
    int attempts = 0;
    unsigned char* curLocation = NULL;
    curLocation = (unsigned char*) buf;

    int found = 0;  // If we found endMsg in the rx buffer, we will mark this value with 1.
    while ((attempts < maxNumAttempts) && (totalBytesRecv < bufsize) ) {
        bytesRecv = SerialRecv(curLocation, bufsize - totalBytesRecv, timeout);
        if (bytesRecv < 0) {
            logError("\tError occurred while reading data from modem, aborting.");
            SerialFlushInputBuff();
            return RET_ERROR;
        }

        totalBytesRecv += bytesRecv;
        if (strstr(buf, endMsg)!=NULL) {
            logInfo("Found `%s`", endMsg);
            found = 1;
            break;
        }
		if (strstr(buf, "ERROR")!=NULL) {
			logError("Found `ERROR`");
			break;
		}
		++attempts;
        curLocation += bytesRecv;
    }

    buf[totalBytesRecv] = '\0';
    logInfo("Received:%s", buf);

    if (!found){
        logError("Error, did not encounter `%s` in incoming data from modem.", endMsg);
        SerialFlushInputBuff();
        return RET_ERROR;
    }

    SerialFlushInputBuff();
    return totalBytesRecv;
}


/************************ useful macros *****************************/

static inline int READ_RX_BUFFER(int bufsize, int timeout, const char* endMsg) {
    return readUntil(RESULT_BUFFER, bufsize, timeout, endMsg, MAX_NUMBER_RETRIES);
}

/**
 * Executes `executeCommand()` and `readUntil` sequentially, resetting RESULT_BUFFER after printing output to stdout.
 */
static int execSimpleModemCommand(const char* cmd, int timeout) {
    int bytesSent = executeCommand(cmd, (int)strlen(cmd));
    if (bytesSent < 0) {
        logError("An error occurred while executing modem command `%s`", cmd);
        return RET_ERROR;
    }

    int bytesRead = READ_RX_BUFFER(50, timeout, OK_END_MSG);
    CLEAR_RX_BUFFER();
    if (bytesRead < 0) {
        logError("An error occurred while reading the result output from execution of command`%s`", cmd);
        return RET_ERROR;
    }

        return bytesRead;
}

static int execCommandWithOutput(const char* cmd, int timeout, const char* endMsg) {
    int bytesSent = executeCommand(cmd, (int)strlen(cmd));
    if (bytesSent < 0) {
        logError("An error occurred while executing modem command `%s`", cmd);
        return RET_ERROR;
    }

    int bytesRead = READ_RX_BUFFER(100, timeout, endMsg);
    CLEAR_RX_BUFFER();
    if (bytesRead < 0) {
        logError("An error occurred while reading the result output from execution of command`%s`", cmd);
        return RET_ERROR;
    }

        return bytesRead;
}


/***************** Implementation of cellular.h functions ***********/

/**
 * Initialize whatever is needed to start working with the cellular modem (e.g. the serial port). Returns 0 on success, and -1 on failure.
 * @param port
 * @return
 */
int CellularInit(char *port) {
    setupTimer();
    logInfo("CellularInit");
    int retval = SerialInit(port, BAUDRATE);
    if (retval < 0) {
        logError("Failed CellularInit, aborting.");
        return RET_ERROR;
    }

    SerialFlushInputBuff();

    CLEAR_RX_BUFFER();
    logInfo("Turning off echo mode");
    static const char* cmdATE0 = "ATE0";
    retval = execSimpleModemCommand(cmdATE0, SHORT_TIMEOUT);
    if (retval == RET_ERROR) {
        logError("A fatal error has occurred, aborting.");
        SerialDisable();
        return RET_ERROR;
    }

    return RET_SUCCESS;
}

int CellularReboot(void){
	static const char* PBREADY = "+PBREADY";
	static const char* cmdCFUN = "AT+CFUN=1,1";
	int retval = execCommandWithOutput(cmdCFUN, MEDIUM_TIMEOUT, PBREADY);
	if (retval < 0) {
		logError("Failed to connect to modem, aborting.");
		SerialDisable();
		return RET_ERROR;
	}
	return RET_SUCCESS;
}

/**
 * Deallocate / close whatever resources CellularInit() allocated.
 */
int CellularDisable(void) {
    logInfo("CellularDisable");
    int retval = CellularClose();
    if (retval < 0) {
        logError("Failed to close cellular socket");
    }
    retval = CellularReboot();
    if (retval <0) {
        logError("Failed to reboot Modem");
    }
    retval = SerialDisable();
    if (retval < 0) {
        logError("An error occurred while disabling serial connection to modem");
    }

    return retval;
}

/**
 * Checks if the modem is responding to AT commands. Return 0 if it does, returns -1 otherwise.
 * @return
 */
int CellularCheckModem(void) {
    static const char* cmdAT = "AT";
    logDebug("Sending `%s` cmd", cmdAT);
    int retval = execSimpleModemCommand(cmdAT, SHORT_TIMEOUT);
    if (retval == RET_ERROR) {
        logError("A fatal error has occurred, aborting.");
        return SerialDisable();

    } else if (retval == RET_WARNING) {
        retval = RET_ERROR;

        logError("Command execution of `%s` failed, skipping.", cmdAT);
    }

    return retval;
}

/**
 * Returns -1 if the modem did not respond or respond with an error.
 * Returns 0 if the command was successful and the registration status was obtained from the modem.
 * In that case, the status parameter will be populated with the numeric value of the <regStatus>field of the ג€�+CREGג€� AT command.
 * @param status
 * @return
 */
int CellularGetRegistrationStatus(int *status) {
    static const char* cmdCREG = "AT+CREG?";
    static const char* hasPrefix = "+CREG: 0,";

    int bytesRecv;
    int bytesSend;

    bytesSend = executeCommand(cmdCREG, (int)strlen(cmdCREG));
    if (bytesSend == RET_ERROR) {
       logError("An error occurred while executing modem command `%s`, aborting", cmdCREG);
        return RET_ERROR;
    }

    bytesRecv = READ_RX_BUFFER(30, SHORT_TIMEOUT, OK_END_MSG);
    if (bytesRecv == RET_ERROR) {
        logError("An error occurred while reading from modem, aborting");
        SerialFlushInputBuff();
        CLEAR_RX_BUFFER();
        return RET_ERROR;
    }

    if (bytesRecv == 0) {
        logError("Failed to read data from mode, got 0 bytes");
        SerialFlushInputBuff();
        CLEAR_RX_BUFFER();
        return RET_ERROR;
    }

    char *prefixPtr = strstr(RESULT_BUFFER, hasPrefix);
    if (strstr(prefixPtr, hasPrefix) == NULL) {
        logError("Failed to parse status code from modem's output");
        SerialFlushInputBuff();
        CLEAR_RX_BUFFER();
        return RET_ERROR;
    }

    prefixPtr += strlen(hasPrefix);
    char *endPtr = NULL;
    int tempStatus = (int)strtol(prefixPtr, &endPtr, 10);
    if (tempStatus < 0 || tempStatus > 5) {
        logError("Failed to parse status code from modem's output");
        SerialFlushInputBuff();
        CLEAR_RX_BUFFER();
        return RET_ERROR;
    }

    *status = tempStatus;
    SerialFlushInputBuff();
    CLEAR_RX_BUFFER();
    return RET_SUCCESS;
}



/**
 * Helper function to CellularGetOperators, parses a single operator string in the format :
 *          1,"Orange IL","OrangeIL","42501",0
 * @param startOp Pointer to string to be parsed
 * @param length Length of startOp
 * @param operator Pointer to OPERATOR_INFO struct that will be filled with the parsed details.
 *                  Assumed to be pre-allocated.
 * @return 0 if successful, -1 otherwise.
 */
static int parseSingleOp(const char* startOp, int length, OPERATOR_INFO *operator) {
    logDebug("Parsing operator string: '%s'", startOp);
    char *seek = NULL;
    seek = strstr(startOp, "\",\""); // ***
    if (seek == NULL) {
        logError("couldn't find the operator name");
        return RET_ERROR;
    }
    seek += 3; // go past ***
    size_t operatorNameLen = strcspn(seek, "\"");
    memset(operator->operatorName, 0, 10);
    strncpy(operator->operatorName, seek, operatorNameLen);
    seek += operatorNameLen + 3; // go past operator name and ***
    char* endPtr = NULL;

    operator->operatorCode = (int)strtol(seek, &endPtr, 10);
    memset(operator->accessTechnology, 0, 5);
    int technology = startOp[length-1] - '0';
    strcpy(operator->accessTechnology, TECHNOLOGIES[technology]);

    return RET_SUCCESS;
}

/**
 * forces the modem to search for available operators (see ג€�+COPS=?ג€� command).
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
int CellularGetOperators(OPERATOR_INFO *opList, int maxops, int *numOpsFound) {
    static const char* cmdAT_COPS_ALL = "AT+COPS=?";
    static const char* prefix = "+COPS: ";
    logDebug("Sending `%s` command to modem and reading output", cmdAT_COPS_ALL);
    int bytesSent = executeCommand(cmdAT_COPS_ALL, (int)strlen(cmdAT_COPS_ALL));
    if (bytesSent < 0) {
        logError("An error occurred while executing command `%s`", cmdAT_COPS_ALL);
        return RET_ERROR;
    }

    int bytesRead = READ_RX_BUFFER(750, LONG_TIMEOUT, OK_END_MSG);
    if (bytesRead < 0) {
        logError("An error occurred while reading the output of command`%s` from modem", cmdAT_COPS_ALL);
        CLEAR_RX_BUFFER();
        return RET_ERROR;
    }

    logDebug("Start parsing operators");
    OPERATOR_INFO operator = {0};
    char *prefixPtr = NULL;
    prefixPtr = strstr(RESULT_BUFFER, prefix);
    if (prefixPtr == NULL) {
        logError("Did not find `+COPS:` prefix");
        CLEAR_RX_BUFFER();
        return RET_ERROR;
    }
    logDebug("Found operators list");
    prefixPtr += strlen(prefix);
    char* startOp = NULL;
    int result = RET_SUCCESS;
    *numOpsFound = 0;
    int currentOpIndex = *numOpsFound;
    static const char* SEPARATORS = "()";
    startOp = strtok(prefixPtr, SEPARATORS);
    for (int i=0; i < maxops; i++) {
        if (startOp == NULL) 
            break;

        result = parseSingleOp(startOp, strlen(startOp), &opList[currentOpIndex]);
        if (result == 0) {
            operator = opList[currentOpIndex];
            ++currentOpIndex;
            logDebug("Operator #%d -  name: '%s'; Operator code: %d; Operator technology: '%s'", currentOpIndex,
                operator.operatorName, operator.operatorCode, operator.accessTechnology);
        }
        else {
            logError("Failed at parsing a single network operator");
            CLEAR_RX_BUFFER();
            return RET_ERROR;
        }

        startOp = strtok(NULL, SEPARATORS);  // to skip the ','
        startOp = strtok(NULL, SEPARATORS);
        
    }

    *numOpsFound = currentOpIndex;
    SerialFlushInputBuff();
    CLEAR_RX_BUFFER();
    return RET_SUCCESS;
}

/**
 * Forces the modem to register/deregister with a network.
 * Returns 0 if the command was successful, returns -1 otherwise.
 * If mode=0, sets the modem to automatically register with an operator (ignores the operatorCode parameter).
 * If mode=1, forces the modem to work with a specific operator, given in operatorCode.
 * If mode=2, deregisters from the network (ignores the operatorCode parameter).
 * See the ג€�+COPS=<mode>,...ג€� command for more details.
 * @param mode
 * @param operatorCode
 * @return
 */
int CellularSetOperator(int mode, OPERATOR_INFO *op) {
	//TODO change operator code to the operator info struct - see that types match.
    static const char* setCMDSimpleFormat = "AT+COPS=%d";
    static const char* setCMDRegisterFormat = "AT+COPS=%d,2,\"%d\"";

    char setCMD[32] = {0}; // includes AT+COPS=<mode> and extra space for operator code if needed
    if (mode == 0 || mode == 2) {
        logDebug("Setting modem to '%s' mode (mode %d)", mode == 0 ? "Auto" : "Disconnect", mode);
        snprintf(setCMD, 20, setCMDSimpleFormat, mode);
    }
    else if (mode == 1) {
        logDebug("Registering to operator %d", op->operatorCode);
        snprintf(setCMD, 20, setCMDRegisterFormat, mode, op->operatorCode);
    }
    else {
        logError("Unsupported mode: %d. Supported modes are {0,1,2}", mode);
        return RET_ERROR;
    }

    int result = execSimpleModemCommand(setCMD, MEDIUM_TIMEOUT);
    if (result < 0) {
        logError("Failed to execute command %s", setCMD);
        return RET_ERROR;
    }
    if (mode == 1){
    	memcpy(&op_info, op, sizeof(OPERATOR_INFO));
    }
    else if (mode==2){
    	memset(&op_info, 0, sizeof(OPERATOR_INFO));
    }
    logDebug("Successfully executed AT+COPS=<mode>[...] command");
    logInfo("refreshModemMetadata");
//    refreshModemMetadata();
    return RET_SUCCESS;
}

/**
 * Returns -1 if the modem did not respond or respond with an error (note, CSQ=99 is also an error!)
 * Returns 0 if the command was successful and the signal quality was obtained from the modem.
 * In that case, the csq parameter will be populated with the numeric value between -113dBm and -51dBm
 * @param csq
 * @return
 */
int CellularGetSignalQuality(int *csq) {
    static const char* cmdAT_CSQ = "AT+CSQ";
    static const char* csqPrefix = "+CSQ: ";
    *csq = -1;  // Assigning initial error value to CSQ
    logInfo("CellularGetSignalQuality");
    logDebug("Executing AT+CSQ");
    int result = executeCommand(cmdAT_CSQ, strlen(cmdAT_CSQ));
    if (result < 0) {
        logError("Failed to execute command %s", cmdAT_CSQ);
        return RET_ERROR;
    }

    logDebug("Reading CSQ output");
    result = READ_RX_BUFFER(100, SHORT_TIMEOUT, OK_END_MSG);
    if (result < 0) {
        logError("Failed to read output of command `%s` or encountered an error", cmdAT_CSQ);
        CLEAR_RX_BUFFER();
        return RET_ERROR;
    }

    logDebug("Looking for prefix `%s`", csqPrefix);
    char *seek = NULL;
    seek = strstr(RESULT_BUFFER, csqPrefix);
    if (seek == NULL) {
        logError("Failed to parse CSQ output");
        CLEAR_RX_BUFFER();
        return RET_ERROR;
    }

    logDebug("Parsing RSSI value");
    char *separator = NULL;
    seek += strlen(csqPrefix);
    int rssi = strtol(seek, &separator, 10);
    logDebug("RSSI value is %d", rssi);
    if (rssi == 99) {
        logError("Error, RSSI value is 99");
        CLEAR_RX_BUFFER();
        return RET_ERROR;
    }

    if (rssi < 0 || rssi > 31) {
        logError("Illegal RSSI return value: %d", rssi);
        CLEAR_RX_BUFFER();
        return RET_ERROR;
    }

    *csq = -113 + (2*rssi);
    logDebug("Calculated CSQ is %ddBm (CSQ = -113 + RSSI*2 , where RSSI=%d)", *csq, rssi);
    CLEAR_RX_BUFFER();
    logInfo("CSQ: %ddBm, RSSI: %d", *csq, rssi);
    return RET_SUCCESS;
}

/**
 * Returns -1 if the modem did not respond or respond with an error.
 * Returns 0 if the command was successful and the ICCID was obtained from the modem.
 * iccid is a pointer to a char buffer, which is allocated by the caller of this function.
 * The buffer size is maxlen chars. The obtained ICCID will be placed into the iccid buffer as a null-terminated string.
 * @param iccid
 * @param maxlen
 * @return
 */
int CellularGetICCID(char* iccid, int maxlen) {
    static const char* cmdCCID = "AT+CCID?";
    static const char* ccidPrefix = "+CCID: ";
    logInfo("CellularGetICCID");
    logDebug("Sending `%s` command to modem and reading output", cmdCCID);
    int bytesSend = executeCommand(cmdCCID, (int)strlen(cmdCCID));
    if (bytesSend == RET_ERROR) {
        logError("A fatal error has occurred, aborting.");
        return SerialDisable();
    } else if (bytesSend == RET_WARNING) {
        logError("Command execution of `%s` failed, skipping.", cmdCCID);
    }

    int bytesRecv = READ_RX_BUFFER(50, SHORT_TIMEOUT, OK_END_MSG);

    if (bytesRecv < 0) {
        logError("An error occurred while reading the output of command`%s`", cmdCCID);
        CLEAR_RX_BUFFER();
        return RET_ERROR;
    }

    memset(iccid, 0, maxlen);
    char *prefixPtr = strstr(RESULT_BUFFER, ccidPrefix);
    if (prefixPtr == NULL) {
        CLEAR_RX_BUFFER();
        logError("Did not find ICCID prefix");
        return RET_ERROR;
    }

    prefixPtr += strlen(ccidPrefix);
    static const char* ALL_DIGITS = "0123456789";
    size_t ccidLength = strspn(prefixPtr, ALL_DIGITS);
    if (ccidLength <= 0) {
        logError("Empty ICCID field");
        CLEAR_RX_BUFFER();
        return RET_ERROR;
    }
    strncpy(iccid, prefixPtr, ccidLength);
    CLEAR_RX_BUFFER();
    logInfo("ICCID: %s", iccid);
    return RET_SUCCESS;
}

/**
 * Returns -1 if the modem did not respond, respond with an error, respond with SEARCH or NOCONN.
 * Returns 0 if the command was successful and the signal info was obtained from the modem.
 * sigInfo is a pointer to a struct, which is allocated by the caller of this function.
 * The obtained info will be placed into the sigInfo struct.
 * @param sigInfo
 * @return
 */
int CellularGetSignalInfo(SIGNAL_INFO *sigInfo) {
    static const char *cmdAT_SMONI = "AT^SMONI";
    static const char *prefixSMONI = "^SMONI: ";

    static const char *IN_TRANSITION_INDICATOR = ",--";
    // Making 5 attempts to make sure Conn_state is not --
    // This can happen when transition from GSM to UMTS networks
    for (int i=0; i < 5; i++) {
        logDebug("Executing '%s'", cmdAT_SMONI);
        int result = executeCommand(cmdAT_SMONI, strlen(cmdAT_SMONI));
        if (result < 0) {
            logError("Error while executing '%s'", cmdAT_SMONI);
            return RET_ERROR;
        }

        logDebug("Reading output for '%s'", cmdAT_SMONI);
        result = READ_RX_BUFFER(50, SHORT_TIMEOUT, OK_END_MSG);
        if (result < 0) {
            logError("Error while reading output from modem");
            CLEAR_RX_BUFFER();
            return RET_ERROR;
        }

        if (strstr(RESULT_BUFFER, IN_TRANSITION_INDICATOR) == NULL)
            break;
    }

    logDebug("Search for prefix '%s'", prefixSMONI);
    char *seek = NULL;
    seek = strstr(RESULT_BUFFER, prefixSMONI);
    if (seek == NULL) {
        logError("");
        CLEAR_RX_BUFFER();
        return RET_ERROR;
    }

    logDebug("Start parsing signal information from output");
    seek += strlen(prefixSMONI);

    logDebug("Parse ACT field (accessTechnology)");
    char *fieldStart = NULL;
    char *fieldEnd = NULL;
    fieldStart = strtok(seek, ",");  // Tokenizing ',' to easily parse the output
    if (fieldStart == NULL) {
        logError("ACT field missing");
        CLEAR_RX_BUFFER();
        return RET_ERROR;
    }

    logDebug("Comparing ACT field to supported technologies");
    memset(sigInfo, 0, sizeof(SIGNAL_INFO));
    int accessTechnology = -1;
    if (strstr(fieldStart, TECHNOLOGIES[GSM_TECH_INDEX]) != NULL) {
        accessTechnology = GSM_TECH_INDEX;
    }
    else if (strstr(fieldStart, TECHNOLOGIES[UMTS_TECH_INDEX]) != NULL) {
        accessTechnology = UMTS_TECH_INDEX;
    }
    else {
        logWarn("Unsupported value for ACT: '%s' , aborting", fieldStart);
        CLEAR_RX_BUFFER();
        return RET_ERROR;
    }
    strcpy(sigInfo->accessTechnology, TECHNOLOGIES[accessTechnology]);
    logDebug("ACT field value: '%s', accessTechnology: '%s'", fieldStart, sigInfo->accessTechnology);
    
    logDebug("Parse signal power");
    if (accessTechnology == GSM_TECH_INDEX) {
        logDebug("Parsing BCCH (signal strength in dBm if GSM)");

        /* BCCH is the third field from the beginning, so we
            search for the first character after the second comma*/
        fieldStart = strtok(NULL, ","); // Now points to second field
        fieldStart = strtok(NULL, ","); // Now points to third field
        if (fieldStart == NULL) {
            logError("Failed to parse BCCH, missing field");
            CLEAR_RX_BUFFER();
            return RET_ERROR;
        }

        sigInfo->signal_power = strtol(fieldStart, &fieldEnd, 10);
        fieldEnd = NULL;
    }

    else if (accessTechnology == UMTS_TECH_INDEX) {
        logDebug("a) Parsing EC/n0 (carrier to noise ratio in dB if UMTS)");
        /* BCCH is the fourth field from the beginning, so we
            search for the first character after the third comma*/
        fieldStart = strtok(NULL, ","); // Now points to second field
        fieldStart = strtok(NULL, ","); // Now points to third field
        fieldStart = strtok(NULL, ","); // Now points to fourth field
        if (NULL == fieldStart) {
            logError("Failed to parse EC_n0, missing field");
            CLEAR_RX_BUFFER();
            return RET_ERROR;
        }

        sigInfo->EC_n0 = strtol(fieldStart, &fieldEnd, 10);
        fieldEnd = NULL;

        logDebug("b) Parsing RCSP (signal power in dBm if UMTS)");
        /* RCSP is the fifth field from the beginning, so we
            search for the first character after the fourth comma*/
        fieldStart = strtok(NULL, ","); // Now points to fifth field
        if (NULL == fieldStart) {
            logError("Failed to parse RCSP, missing field");
            CLEAR_RX_BUFFER();
            return RET_ERROR;
        }

        sigInfo->signal_power = strtol(fieldStart, &fieldEnd, 10);
        fieldEnd = NULL;
    }
    else {
        // Shouldn't be able to reach, but just in case
        logError("Unrecognized cellular access technology, aborting");
        CLEAR_RX_BUFFER();
        return RET_ERROR;
    }

    logDebug("Finished parsing signal information: accessTechnology=%s, signal_power=%ddBm, EC/n0=%ddB",
        sigInfo->accessTechnology, sigInfo->signal_power, sigInfo->EC_n0);

    logDebug("Parsing connection state");
    static const char* SEPARATOR = ",";
    static const char* NOCONN = "NOCONN";
    static const char* SEARCH = "SEARCH";
    static int CONN_STATE_LEN = 6; 
    char *connState = NULL;

    logDebug("Iterating over fields until reaching the final field");
    logDebug("Current field: %s", fieldStart);
    fieldStart = strtok(NULL, SEPARATOR);
    while (fieldStart != NULL) {
        logDebug("Current field: %s", fieldStart);
        fieldEnd = strtok(NULL, SEPARATOR);

        if (fieldStart != NULL && fieldEnd == NULL) {
            // This indicates fieldStart is the last field
            connState = fieldStart;
        }
        fieldStart = strtok(NULL, SEPARATOR);
    }
    int lastFieldLength = 0;
    lastFieldLength = strcspn(connState, " \t\r\n");
    connState[lastFieldLength] = '\0'; // Truncate
 
    // Search backwards, not necessarily an error if no connection state in string
    if (connState != NULL) {
        if (strncmp(connState, NOCONN, CONN_STATE_LEN) == 0 || strncmp(connState, SEARCH, CONN_STATE_LEN) == 0) {
            logWarn("Detected '%s' state, aborting", connState);
            CLEAR_RX_BUFFER();
            return RET_ERROR;
        }

        logDebug("Value of Conn_state: '%s'", connState);
    }

    CLEAR_RX_BUFFER();
    return RET_SUCCESS;
}

/**
 * Initialize an internet connection profile (AT^SICS) with inactTO=inact_time_sec and
 * conType=GPRS0 and apn="postm2m.lu". Return 0 on success, and -1 on failure.
 * @param inact_time_sec
 * @return 0 on success, -1 on failure
 */
int CellularSetupInternetConnectionProfile(int inact_time_sec) {
    static const char* AT_SICS_CON_TYPE = "AT^SICS=0,conType,\"GPRS0\"";
    static const char* AT_SICS_APN = "AT^SICS=0,apn,\"postm2m.lu\"";
    static const char* AT_SICS_INACT_TO = "AT^SICS=0,inactTO,\"%d\"";
    static char AT_SICS_INACT_TO_CMD[64] = {0};
    
    logDebug("Executing CellularSetupInternetConnectionProfile()");
    int bytesRecv = execCommandWithOutput(AT_SICS_CON_TYPE, SHORT_TIMEOUT, OK_END_MSG);
    if (bytesRecv < 0){
        logError("Command %s failed. exiting", AT_SICS_CON_TYPE);
        return RET_ERROR;
    }
    bytesRecv = execCommandWithOutput(AT_SICS_APN, SHORT_TIMEOUT, OK_END_MSG);
    if (bytesRecv < 0){
        logError("Command %s failed. exiting", AT_SICS_APN);
        return RET_ERROR;
    }
    sprintf(AT_SICS_INACT_TO_CMD, AT_SICS_INACT_TO, inact_time_sec);
    bytesRecv = execCommandWithOutput(AT_SICS_INACT_TO_CMD, SHORT_TIMEOUT, OK_END_MSG);
    if (bytesRecv < 0){
        logError("Command %s failed. exiting", AT_SICS_INACT_TO_CMD);
        return RET_ERROR;
    }
    return RET_SUCCESS;
}

/**
 * Initialize an internal service profile (AT^SISS) with keepintvl=keepintvl_sec (the timer) and
 * SrvType=Socket, and conId=<CellularSetupInternetConnectionProfile_id>
 * (if CellularSetupInternetConnectionProfile is already initialized. Return error, -1, otherwise) and
 * Address=socktcp://IP:port;AT_SICS_INACT_TO;time=keepintvl_sec. Return 0 on success, and -1 on failure.
 * @param IP
 * @param port
 * @param keepintvl_sec
 * @return 0 on success, -1 on failure
 */
int CellularSetupInternetServiceSetupProfile(const char *IP, unsigned int port, int keepintvl_sec){
    static const char* AT_SISS_SERV_TYPE = "AT^SISS=1,\"SrvType\",\"Socket\"";
    static const char* AT_SISS_CON_ID = "AT^SISS=1,\"conId\",\"0\"";
    static const char* AT_SISS_ADDR = "AT^SISS=1,\"address\",\"socktcp://%s:%d;etx;timer=%d\"";
    static char AT_SISS_ADDR_CMD[128] = {0};
    
    logDebug("Executing CellularSetupInternetServiceSetupProfile()");    
    int bytesRecv = execCommandWithOutput(AT_SISS_SERV_TYPE, SHORT_TIMEOUT, OK_END_MSG);
    if (bytesRecv < 0){
        logError("Command %s failed. exiting", AT_SISS_SERV_TYPE);
        return RET_ERROR;
    }
    bytesRecv = execCommandWithOutput(AT_SISS_CON_ID, SHORT_TIMEOUT, OK_END_MSG);
    if (bytesRecv < 0){
        logError("Command %s failed. exiting", AT_SISS_CON_ID);
        return RET_ERROR;
    }
    int cmdLen = sprintf(AT_SISS_ADDR_CMD,AT_SISS_ADDR, IP, port, keepintvl_sec);
    AT_SISS_ADDR_CMD[cmdLen] = '\0';

    bytesRecv = execCommandWithOutput(AT_SISS_ADDR_CMD, SHORT_TIMEOUT, OK_END_MSG);
    if (bytesRecv < 0){
        logError("Command %s failed. exiting", AT_SISS_ADDR_CMD);
        return RET_ERROR;
    }
    return RET_SUCCESS;
}

static int g_isConnected = 0;
static int g_inSilentMode = 0;

#ifdef CHECK_SOCKET_COMM
static int checkSocketCommunicationSt(){
  static const char* AT_SISI="AT^SISI?";
  static const char* hasPrefix ="^SISI: 1,";
  int bytesSend = executeCommand(AT_SISI, (int)strlen(AT_SISI));
  if (bytesSend == RET_ERROR) {
      logError("An error occurred while executing modem command `%s`, aborting", AT_SISI);
      return RET_ERROR;
  }

  int bytesRecv = READ_RX_BUFFER(64, MEDIUM_TIMEOUT, OK_END_MSG);
  if (bytesRecv == RET_ERROR) {
      logError("An error occurred while reading from modem, aborting");
      SerialFlushInputBuff();
      CLEAR_RX_BUFFER();
      return RET_ERROR;
  }
  int* status = NULL;
  char *prefixPtr = strstr(RESULT_BUFFER, hasPrefix);
  if (strstr(prefixPtr, hasPrefix) == NULL) {
      logError("Failed to parse status code from modem's output");
      SerialFlushInputBuff();
      CLEAR_RX_BUFFER();
      return RET_ERROR;
  }

  prefixPtr += strlen(hasPrefix);
  char *endPtr = NULL;
  int tempStatus = (int)strtol(prefixPtr, &endPtr, 10);
  if (tempStatus < 0 || tempStatus > 9) {
      logError("Failed to parse status code from modem's output");
      SerialFlushInputBuff();
      CLEAR_RX_BUFFER();
      return RET_ERROR;
  }

  *status = tempStatus;
  SerialFlushInputBuff();
  CLEAR_RX_BUFFER();


  if ((*status == 3)||(*status == 4)||(*status == 8)){
      logInfo("socket connected properly");
      return RET_SUCCESS;
  }
  return RET_ERROR;
}
#endif // CHECK_SOCKET_COMM

/**
 * Connects to the socket (establishes TCP connection to the pre-defined host and port).
 * @return 0 on success, -1 on failure
 */
int CellularConnect(void) {
    static const char* AT_SISC = "AT^SISC=1";
    static const char* AT_SISO = "AT^SISO=1";
    static const char* AT_SISE = "AT^SISE=1";
    static const char* AT_SIST = "AT^SIST=1";

    logDebug("Executing CellularConnect()");
    if (g_isConnected) {
        logDebug("Already connected");
        return RET_SUCCESS;
    }

    logDebug("Closing socket if it was previously open");
    int bytesRecv = execCommandWithOutput(AT_SISC, SHORT_TIMEOUT, OK_END_MSG);
    if (bytesRecv < 0) {
        logWarn("warn: no '%s' after `%s`",OK_END_MSG, AT_SISC);
    }

    logDebug("Opening modem TCP connection");
    bytesRecv = execCommandWithOutput(AT_SISO, SHORT_TIMEOUT, OK_END_MSG);
    if (bytesRecv < 0) {
	logError("%s Failed, closing socket", AT_SISO);
        return RET_ERROR;
    }
    g_isConnected = 1;

	sleepMs(5000);
    logDebug("Attempting to enter silent TCP mode");
    bytesRecv = execCommandWithOutput(AT_SIST, SHORT_TIMEOUT, "CONNECT");
    if (bytesRecv < 0) {
        g_inSilentMode = 0;
        logError("%s Failed, closing socket", AT_SIST);
        logError("Error reason:");
        execCommandWithOutput(AT_SISE, SHORT_TIMEOUT, OK_END_MSG);
        return RET_ERROR;
    }

    g_inSilentMode = 1;
    return RET_SUCCESS;
}


/**
 * Closes the established connection.
 * @return 0 on success, -1 on failure
 */
int CellularClose(){
    static const char* CMD_CLOSE_TCP_CLIENT = "+++";
    static const char* AT_SISC = "AT^SISC=1";

    logInfo("CellularClose");
    sleepMs(1200);
    int bytesRecv;

    int bytesSent = SerialSend((unsigned char*)CMD_CLOSE_TCP_CLIENT, 3);
    logDebug("Writing escape sequence '%s'", CMD_CLOSE_TCP_CLIENT);
    if (bytesSent < 0) {
        logError("escape sequence '%s' write failed", CMD_CLOSE_TCP_CLIENT);
    }
    else {
        logDebug("Wrote %d bytes to serial port", bytesSent);
        bytesRecv = readUntil(RESULT_BUFFER, 20, 200, OK_END_MSG, MAX_NUMBER_RETRIES);
        if (bytesRecv < 0) {
            logWarn("escape sequence '%s' failed", CMD_CLOSE_TCP_CLIENT);
        }
    }
    sleepMs(1200);

    g_inSilentMode = 0;


    logDebug("Closing cellular modem's TCP connection");
    bytesRecv = execCommandWithOutput(AT_SISC, SHORT_TIMEOUT, OK_END_MSG);
    if (bytesRecv < 0) {
        logWarn("Didn't receive %s after command: `%s`",OK_END_MSG, AT_SISC);
    }

    if (CellularCheckModem() < 0){
        logError("CellularCheckModem fail");
        return RET_ERROR;

    }

    g_isConnected = 0;
    return RET_SUCCESS;
}

/**
 * Writes len bytes from payload buffer to the established connection.
 * @param payload
 * @param len
 * @return number of bytes on success, -1 on failure
 */
int CellularWrite(const unsigned char *payload, unsigned int len){
    logInfo("CellularWrite %d bytes", len);
    if (!g_inSilentMode) {
        logError("Failed to write, no connection to modem");
        return RET_ERROR;
    }

    int bytesWritten = 0;
    int bytesRemaining = len;
    int bytesToWrite;
    int bytesSent;
    while (bytesRemaining > 0) {
        bytesToWrite = bytesRemaining > _TX_MODEM_BUFFER_SIZE ? _TX_MODEM_BUFFER_SIZE : bytesRemaining;
        logDebug("Writing %d bytes to modem. Payload='%.*s'",bytesToWrite, bytesToWrite, &payload[bytesWritten]);
        bytesSent = SerialSend(&payload[bytesWritten], bytesToWrite);
        bytesSent = bytesToWrite;
        if (bytesSent < 0){
            logError("CellularWrite failed in middle after %d/%d bytes.", bytesWritten, len);
            return RET_ERROR;
        }
        bytesRemaining -= bytesSent;
        bytesWritten += bytesSent;
    }

    return bytesWritten;
}

/**
 * Reads up to max_len bytes from the established connection to the provided buf buffer, for up to timeout_ms (doesn't block
 * longer than that, even if not all max_len bytes were received).
 * @param buf
 * @param max_len
 * @param timeout_ms
 * @return number of bytes on success, -1 on failure
 */
int CellularRead(unsigned char *buf, unsigned int max_len, unsigned int timeout_ms) {
    logInfo("CellularRead (max %d bytes)", max_len);
    if (!g_inSilentMode) {
        logError("Failed to read, no connection to modem");
        return RET_ERROR;
    }

    int bytesRecv = SerialRecv(buf, max_len, timeout_ms);
    if (bytesRecv < 0){
        logError("Unable to send read from modem.");
        return RET_ERROR;
    }

    logInfo("CellularRead success: Read %d bytes", bytesRecv);
    return bytesRecv;
}

static void refreshModemMetadata(void){
	memset(&cellularMetaData, 0, sizeof(cellularMetaData));
	memcpy(&cellularMetaData.op_info, &op_info,sizeof(OPERATOR_INFO));
	CellularGetICCID(cellularMetaData.ccid, ICCID_NUM_DIGITS);
	CellularGetSignalQuality(&cellularMetaData.csq);
}

void inline GetModemMetadata(MODEM_METADATA *metaData)
{
	memcpy(metaData, &cellularMetaData,sizeof(MODEM_METADATA));
}

