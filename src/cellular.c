#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "timer_manager.h"
#include "cellular.h"
#include "serial_io.h"
#include "logger.h"

/******* Instantiate data structures, constants, and type definitions ********/
#define _RX_MODEM_BUFFER_SIZE 4096
#define _TX_MODEM_BUFFER_SIZE 100
#define GSM_TECH_INDEX 0
#define UMTS_TECH_INDEX 2

enum RETURN_CODE {RET_SUCCESS = 0, RET_ERROR = -1, RET_WARNING = -2};
static char CMD_BUFFER[_TX_MODEM_BUFFER_SIZE] = {0};
static char RESULT_BUFFER[_RX_MODEM_BUFFER_SIZE] = {0};

#define CLEAR_RX_BUFFER() memset(RESULT_BUFFER, 0, _RX_MODEM_BUFFER_SIZE)
#define CLEAR_TX_BUFFER() memset(CMD_BUFFER, 0, _TX_MODEM_BUFFER_SIZE)

static const char* TECHNOLOGIES[3] = {"2G", "", "3G"};
static const char* OK_END_MSG = "OK";

static int modemHandle = -1;
static int g_isConnected = 0;
static int g_inSilentMode = 0;

/************************ utility functions *****************************/

/**
 * Executes `command` on modem
 * @param command Command to execute
 * @param commandLen Command length
 * @return ERROR if a fatal error has occurred, WARNING if a non fatal error occurred, and number of sent bytes if successful.
 */
static int executeCommand(const char* command, int commandLen)
{
  logInfo("executeCommand `%s`", command);
  int commandLenUpdated = commandLen + (int)strlen(SERIAL_ENDL);
  CLEAR_TX_BUFFER();
  // Pre-formatting command
  snprintf(CMD_BUFFER, commandLenUpdated + 1, "%s%s%c", command, SERIAL_ENDL, '\0');
  int bytesSent = SerialSend((unsigned char*)CMD_BUFFER, commandLenUpdated+1, &modemHandle);
  CMD_BUFFER[0]='\0';
  if (bytesSent < 0)
  {
    logError("executeCommand failed");
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
  while ((attempts < maxNumAttempts) && (totalBytesRecv < bufsize))
  {
    bytesRecv = SerialRecv(curLocation, bufsize - totalBytesRecv, timeout, &modemHandle);
    if (bytesRecv < 0)
    {
      logError("readUntil fail at SerialRecv");
      SerialFlushInputBuff(&modemHandle);
      return RET_ERROR;
    }

    totalBytesRecv += bytesRecv;
    if (strstr(buf, endMsg) != NULL)
    {
      logDebug("Found `%s`", endMsg);
      found = 1;
      break;
    }

    if (strstr(buf, "ERROR") != NULL) {
      logError("Found `ERROR`");
      break;
    }
    ++attempts;
    curLocation += bytesRecv;
  }

  buf[totalBytesRecv] = '\0';
  logDebug("Received:%s", buf);
  if (!found){
    logError("readUntil fail, output missing `%s`", endMsg);
    SerialFlushInputBuff(&modemHandle);
    return RET_ERROR;
  }

  SerialFlushInputBuff(&modemHandle);
  return totalBytesRecv;
}


/************************ useful macros *****************************/

static inline int READ_RX_BUFFER(int bufsize, int timeout, const char* endMsg) {
  return readUntil(RESULT_BUFFER, bufsize, timeout, endMsg, MAX_NUMBER_RETRIES);
}

/**
 * Executes `executeCommand()` and `readUntil` sequentially, resetting RESULT_BUFFER after printing output to stdout.
 */
static int execCommandWithOutput(const char* cmd, int timeout, const char* endMsg) {
  int bytesSent = executeCommand(cmd, (int)strlen(cmd));
  if (bytesSent < 0)
  {
    logError("An error occurred while executing modem command `%s`", cmd);
    return RET_ERROR;
  }

  int bytesRead = READ_RX_BUFFER(100, timeout, endMsg);
  CLEAR_RX_BUFFER();
  if (bytesRead < 0)
  {
    logError("An error occurred while reading the result output from execution of command`%s`", cmd);
    return RET_ERROR;
  }

  return bytesRead;
}


/***************** Implementation of cellular.h functions ***********/

int CellularInit(void)
{
  logInfo("CellularInit");
  setupTimer();
  int retval = SerialInit(MODEM_PORT, MODEM_BAUDRATE, &modemHandle);
  if (retval < 0)
  {
    logError("CellularInit failed SerialInit");
    return retval;
  }

  SerialFlushInputBuff(&modemHandle);

  CLEAR_RX_BUFFER();
  logDebug("Turning off echo mode");
  static const char* cmdATE0 = "ATE0";
  retval = execCommandWithOutput(cmdATE0, SHORT_TIMEOUT, OK_END_MSG);
  if (retval < 0)
  {
    logError("CellularInit failed ATE0");
    SerialDisable(&modemHandle);
    return retval;
  }

  return RET_SUCCESS;
}

int CellularReboot(void)
{
  static const char* PBREADY = "+PBREADY";
  static const char* cmdCFUN = "AT+CFUN=1,1";
  int retval = execCommandWithOutput(cmdCFUN, MEDIUM_TIMEOUT, PBREADY);
  if (retval < 0)
  {
    logError("CellularReboot fail, no PBREADY");
    SerialDisable(&modemHandle);
    return retval;
  }

  return RET_SUCCESS;
}

int CellularDisable(void)
{
  logInfo("CellularDisable");
  int retval = CellularClose();
  if (retval < 0)
  {
    logError("CellularDisable fail at CellularClose");
  }

  retval = CellularReboot();
  if (retval <0)
  {
    logError("Failed to reboot Modem");
  }

  retval = SerialDisable(&modemHandle);
  if (retval < 0)
  {
    logError("CellularDisable fail at SerialDisable");
    retval = -1;
  }

  modemHandle = -1;
  return retval;
}

int CellularCheckModem(void)
{
  static const char* cmdAT = "AT";
  logInfo("CellularCheckModem");
  int retval = execCommandWithOutput(cmdAT, SHORT_TIMEOUT, OK_END_MSG);
  if (retval < 0)
  {
    logError("CellularCheckModem fail, no OK");
  }

  return RET_SUCCESS;
}

int CellularGetRegistrationStatus(int *status)
{
  static const char* cmdCREG = "AT+CREG?";
  static const char* hasPrefix = "+CREG: 0,";

  int bytesSend = executeCommand(cmdCREG, (int)strlen(cmdCREG));
  if (bytesSend < 0)
  {
    logError("%s execution failed", cmdCREG);
    return bytesSend;
  }

  int bytesRecv = READ_RX_BUFFER(30, SHORT_TIMEOUT, OK_END_MSG);
  if (bytesRecv <= 0) {
    logError("%s failed, no OK", cmdCREG);
    SerialFlushInputBuff(&modemHandle);
    CLEAR_RX_BUFFER();
    return bytesRecv;
  }

  char *prefixPtr = strstr(RESULT_BUFFER, hasPrefix);
  if (strstr(prefixPtr, hasPrefix) == NULL)
  {
    logError("CellularGetRegistrationStatus failed parsing");
    SerialFlushInputBuff(&modemHandle);
    CLEAR_RX_BUFFER();
    return RET_ERROR;
  }

  prefixPtr += strlen(hasPrefix);
  char *endPtr = NULL;
  int tempStatus = (int)strtol(prefixPtr, &endPtr, 10);
  if (tempStatus < 0 || tempStatus > 5)
  {
    logError("CellularGetRegistrationStatus failed parsing");
    SerialFlushInputBuff(&modemHandle);
    CLEAR_RX_BUFFER();
    return RET_ERROR;
  }

  *status = tempStatus;
  SerialFlushInputBuff(&modemHandle);
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
static int parseSingleOp(const char* startOp, int length, OPERATOR_INFO *operator)
{
  logDebug("Parsing operator string: '%s'", startOp);
  char *seek = NULL;
  seek = strstr(startOp, "\",\""); // Separator is exactly `","`
  if (seek == NULL)
  {
    logError("parseSingleOp fail, no opName");
    return RET_ERROR;
  }

  seek += 3; // go past `","`
  size_t operatorNameLen = strcspn(seek, "\"");
  memset(operator->operatorName, 0, 10);
  strncpy(operator->operatorName, seek, operatorNameLen);
  seek += operatorNameLen + 3; // go past operator name and `","`
  char* endPtr = NULL;

  operator->operatorCode = (int)strtol(seek, &endPtr, 10);
  memset(operator->accessTechnology, 0, 5);
  int technology = startOp[length-1] - '0';
  strcpy(operator->accessTechnology, TECHNOLOGIES[technology]);

  return RET_SUCCESS;
}

int CellularGetOperators(OPERATOR_INFO *opList, int maxops, int *numOpsFound)
{
  static const char* cmdAT_COPS_ALL = "AT+COPS=?";
  static const char* prefix = "+COPS: ";
  logInfo("CellularGetOperators");
  logDebug("Sending `%s` command to modem and reading output", cmdAT_COPS_ALL);
  int bytesSent = executeCommand(cmdAT_COPS_ALL, (int)strlen(cmdAT_COPS_ALL));
  if (bytesSent < 0)
  {
    logError("CellularGetOperators failed `%s`", cmdAT_COPS_ALL);
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
  if (prefixPtr == NULL)
  {
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
  for (int i=0; i < maxops; i++)
  {
    if (startOp == NULL)
    {
      break;
    }

    result = parseSingleOp(startOp, strlen(startOp), &opList[currentOpIndex]);
    if (result == 0)
    {
      operator = opList[currentOpIndex];
      ++currentOpIndex;
      logDebug("Operator #%d -  name: '%s'; Operator code: %d; Operator technology: '%s'", currentOpIndex,
               operator.operatorName, operator.operatorCode, operator.accessTechnology);
    }
    else
    {
      logError("Failed at parsing a single network operator");
      CLEAR_RX_BUFFER();
      return RET_ERROR;
    }

    startOp = strtok(NULL, SEPARATORS);  // to skip the ','
    startOp = strtok(NULL, SEPARATORS);
  }

  *numOpsFound = currentOpIndex;
  SerialFlushInputBuff(&modemHandle);
  CLEAR_RX_BUFFER();
  return RET_SUCCESS;
}

int CellularSetOperator(int mode, OPERATOR_INFO *op)
{
  static const char* setCMDSimpleFormat = "AT+COPS=%d";
  static const char* setCMDRegisterFormat = "AT+COPS=%d,2,\"%d\"";
  logInfo("CellularSetOperator");

  char setCMD[32] = {0}; // includes AT+COPS=<mode> and extra space for operator code if needed
  if (mode == 0 || mode == 2)
  {
    logDebug("Set modem to '%s' mode (%d)", mode == 0 ? "Auto" : "Disconnect", mode);
    snprintf(setCMD, 20, setCMDSimpleFormat, mode);
  }
  else if (mode == 1)
  {
    logDebug("Registering to operator %d", op->operatorCode);
    snprintf(setCMD, 20, setCMDRegisterFormat, mode, op->operatorCode);
  }
  else
  {
    logError("CellularSetOperator unsupported mode: %d", mode);
    return RET_ERROR;
  }

  int result = execCommandWithOutput(setCMD, MEDIUM_TIMEOUT, OK_END_MSG);
  if (result < 0)
  {
    logError("CellularSetOperator cmd fail, no OK");
    return RET_ERROR;
  }

  logDebug("Successfully executed AT+COPS=<mode>[...] command");
  return RET_SUCCESS;
}

int CellularSetupInternetConnectionProfile(int inact_time_sec)
{
  static const char* AT_SICS_CON_TYPE = "AT^SICS=0,conType,\"GPRS0\"";
  static const char* AT_SICS_APN = "AT^SICS=0,apn,\"postm2m.lu\"";
  static const char* AT_SICS_INACT_TO = "AT^SICS=0,inactTO,\"%d\"";
  static char AT_SICS_INACT_TO_CMD[64] = {0};

  logInfo("CellularSetupInternetConnectionProfile");
  int bytesRecv = execCommandWithOutput(AT_SICS_CON_TYPE, SHORT_TIMEOUT, OK_END_MSG);
  if (bytesRecv < 0)
  {
    logError("Command %s failed", AT_SICS_CON_TYPE);
    return bytesRecv;
  }

  bytesRecv = execCommandWithOutput(AT_SICS_APN, SHORT_TIMEOUT, OK_END_MSG);
  if (bytesRecv < 0)
  {
    logError("Command %s failed", AT_SICS_APN);
    return bytesRecv;
  }

  sprintf(AT_SICS_INACT_TO_CMD, AT_SICS_INACT_TO, inact_time_sec);
  bytesRecv = execCommandWithOutput(AT_SICS_INACT_TO_CMD, SHORT_TIMEOUT, OK_END_MSG);
  if (bytesRecv < 0)
  {
    logError("Command %s failed", AT_SICS_INACT_TO_CMD);
    return bytesRecv;
  }

  return RET_SUCCESS;
}

int CellularSetupInternetServiceSetupProfile(const char *IP, unsigned int port, int keepintvl_sec)
{
  static const char* AT_SISS_SERV_TYPE = "AT^SISS=1,\"SrvType\",\"Socket\"";
  static const char* AT_SISS_CON_ID = "AT^SISS=1,\"conId\",\"0\"";
  static const char* AT_SISS_ADDR = "AT^SISS=1,\"address\",\"socktcp://%s:%d;etx;timer=%d\"";
  static char AT_SISS_ADDR_CMD[128] = {0};

  logInfo("CellularSetupInternetServiceSetupProfile");    
  int bytesRecv = execCommandWithOutput(AT_SISS_SERV_TYPE, SHORT_TIMEOUT, OK_END_MSG);
  if (bytesRecv < 0)
  {
    logError("Command %s failed", AT_SISS_SERV_TYPE);
    return bytesRecv;
  }

  bytesRecv = execCommandWithOutput(AT_SISS_CON_ID, SHORT_TIMEOUT, OK_END_MSG);
  if (bytesRecv < 0)
  {
    logError("Command %s failed", AT_SISS_CON_ID);
    return bytesRecv;
  }

  int cmdLen = sprintf(AT_SISS_ADDR_CMD,AT_SISS_ADDR, IP, port, keepintvl_sec);
  AT_SISS_ADDR_CMD[cmdLen] = '\0';
  bytesRecv = execCommandWithOutput(AT_SISS_ADDR_CMD, SHORT_TIMEOUT, OK_END_MSG);
  if (bytesRecv < 0)
  {
    logError("Command %s failed. exiting", AT_SISS_ADDR_CMD);
    return bytesRecv;
  }

  return RET_SUCCESS;
}

int CellularConnect(void) {
  static const char* AT_SISC = "AT^SISC=1";
  static const char* AT_SISO = "AT^SISO=1";
  static const char* AT_SISE = "AT^SISE=1";
  static const char* AT_SIST = "AT^SIST=1";

  logInfo("CellularConnect");
  if (g_isConnected)
  {
    logDebug("Already connected");
    return RET_SUCCESS;
  }

  logDebug("Closing socket if it was previously open");
  int bytesRecv = execCommandWithOutput(AT_SISC, SHORT_TIMEOUT, OK_END_MSG);
  if (bytesRecv < 0)
  {
    logWarn("CellularConnect warn: no '%s' after `%s`",OK_END_MSG, AT_SISC);
  }

  logDebug("Opening modem TCP connection");
  bytesRecv = execCommandWithOutput(AT_SISO, SHORT_TIMEOUT, OK_END_MSG);
  if (bytesRecv < 0)
  {
    logError("%s Failed, closing socket", AT_SISO);
    logError("Error reason:");
    execCommandWithOutput(AT_SISE, SHORT_TIMEOUT, OK_END_MSG);
    return RET_ERROR;
  }

  g_isConnected = 1;

  sleepMs(5000);
  logDebug("Attempting to enter silent TCP mode");
  bytesRecv = execCommandWithOutput(AT_SIST, SHORT_TIMEOUT, "CONNECT");
  if (bytesRecv < 0)
  {
    g_inSilentMode = 0;
    logError("%s failed", AT_SIST);
    logError("Error reason:");
    execCommandWithOutput(AT_SISE, SHORT_TIMEOUT, OK_END_MSG);
    return RET_ERROR;
  }

  g_inSilentMode = 1;
  return RET_SUCCESS;
}

int CellularClose()
{
  static const char* CMD_CLOSE_TCP_CLIENT = "+++";
  static const char* AT_SISC = "AT^SISC=1";

  logInfo("CellularClose");

  sleepMs(1200);
  int bytesRecv;
  logInfo("Sending escape sequence '%s'", CMD_CLOSE_TCP_CLIENT);
  SerialSend((unsigned char*)CMD_CLOSE_TCP_CLIENT, 3, &modemHandle);

  bytesRecv = readUntil(RESULT_BUFFER, 20, 200, OK_END_MSG, MAX_NUMBER_RETRIES);
  if (bytesRecv < 0)
  {
    logDebug("escape sequence '%s' failed", CMD_CLOSE_TCP_CLIENT);
  }
  sleepMs(1200);
  g_inSilentMode = 0;

  logDebug("Closing cellular modem's TCP connection");
  bytesRecv = execCommandWithOutput(AT_SISC, SHORT_TIMEOUT, OK_END_MSG);
  if (bytesRecv < 0) {
    logWarn("No %s after command `%s`",OK_END_MSG, AT_SISC);
  }

  if (CellularCheckModem() < 0)
  {
    logError("CellularCheckModem fail after SISC");
    return RET_ERROR;
  }

  g_isConnected = 0;
  return RET_SUCCESS;
}

int CellularWrite(const unsigned char *payload, unsigned int len)
{
  logDebug("CellularWrite %d bytes", len);
  if (!g_inSilentMode)
  {
    logError("FCellularWrite fail, no connection");
    return RET_ERROR;
  }

  int bytesWritten = 0;
  int bytesRemaining = len;
  int bytesToWrite;
  int bytesSent;
  while (bytesRemaining > 0)
  {
    bytesToWrite = bytesRemaining > _TX_MODEM_BUFFER_SIZE ? _TX_MODEM_BUFFER_SIZE : bytesRemaining;
    logDebug("Writing %d bytes to modem. Payload='%.*s'",bytesToWrite, bytesToWrite, &payload[bytesWritten]);
    bytesSent = SerialSend(&payload[bytesWritten], bytesToWrite, &modemHandle);
    bytesSent = bytesToWrite;
    if (bytesSent < 0)
    {
      logError("CellularWrite failed in middle after %d/%d bytes.", bytesWritten, len);
      return RET_ERROR;
    }

    bytesRemaining -= bytesSent;
    bytesWritten += bytesSent;
  }

  return bytesWritten;
}

int CellularRead(unsigned char *buf, unsigned int max_len, unsigned int timeout_ms)
{
  logDebug("CellularRead (<=%d bytes)", max_len);
  if (!g_inSilentMode)
  {
    logError("CellularRead fail, no connection");
    return RET_ERROR;
  }

  int bytesRecv = SerialRecv(buf, max_len, timeout_ms, &modemHandle);
  if (bytesRecv < 0)
  {
    logError("CellularRead fail at SerialRecv");
    return RET_ERROR;
  }

  logDebug("CellularRead success: Read %d bytes", bytesRecv);
  return bytesRecv;
}
