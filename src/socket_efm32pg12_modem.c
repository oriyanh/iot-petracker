#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "socket.h"
#include "cellular.h"
#include "logger.h"
#include "timer_manager.h"
#include "wolfmqtt/mqtt_types.h"
#include "stdbool.h"

#define MAX_NUM_OPERATORS 10

static bool registerToNetwork = true;

/**
 * Populates opList with information of all available cellular networks
 */
static int detectCellularNetworks(OPERATOR_INFO* opList, int *numOpsFound);

/**
 * Attempts to register to the first network operator in opList.
 * If that fails, continues to the next. If all attempts fail, returns -1;
 */
static int registerCellularNetwork(OPERATOR_INFO* opList, int numOpsFound);

/**
 * Registers to a cellular network, wrapper function for detectCellularNetworks and registerCellularNetwork
 */
static int registeringToNetworkFlow();

/**
 * Runs cellularInit->CheckModem->detectNetworks->register->refresh. This function is called in SocketInit
 */
static int cellularConnectFlow(void);

int SocketInit(const char *host, word16 port)
{
  logInfo("SocketInit(%s:%d)", host, port);
  int result = cellularConnectFlow();
  if (result < 0) {
    logError("SocketInit failed cellularConnectFlow");
    return result;
  }

  result = CellularSetupInternetConnectionProfile(20);
  if (result < 0)
  {
    logError("CellularSetupInternetConnectionProfile failed");
    return result;
  }

  result = CellularSetupInternetServiceSetupProfile(host, (unsigned int)port, 200);
  if (result < 0)
  {
    logError("CellularSetupInternetServiceSetupProfile failed");
    return result;
  }

  logInfo("SocketInit success");
  return result;
}

int SocketConnect(void)
{
  logInfo("SocketConnect");
  int result = CellularConnect();
  if (result < 0)
  {
    logError("SocketConnect fails at CellularConnect");
    CellularClose();
    return result;
  }

  logInfo("SocketConnect success");
  return 0;
}

int SocketWrite(const unsigned char *payload, unsigned int len)
{
  logInfo("SocketWrite %d bytes", len);

  int numBytesSent = CellularWrite(payload , len);
  if( numBytesSent < 0)
  {
    logError("Failed SocketWrite at CellularWrite");
  }
  else
  {
    logDebug("Successfully wrote %d bytes to socket", numBytesSent);    
  }

  return numBytesSent;
}

int SocketRead(unsigned char *buf, unsigned int max_len, unsigned int timeout_ms)
{
  logInfo("SocketRead %u bytes,timeout_ms=%u", max_len, timeout_ms);
  int numBytesReceived = CellularRead(buf,max_len,timeout_ms);
  if(numBytesReceived < 0)
  {
    logError("Failed SocketRead at CellularRead");
  }

  else
  {
    logInfo("SocketRead success, read %d bytes", numBytesReceived);
  }

  return numBytesReceived;
}

int SocketClose()
{
  logInfo("SocketClose");
  return CellularClose();
}

int SocketDisable()
{
  logInfo("SocketDisable");
  registerToNetwork = true;
  int retval = CellularDisable();
  if (retval < 0)
  {
    logError("SocketDisable failed CellularDisable, skipping to cellularConnectFlow");
  }

  retval = cellularConnectFlow();
  if (retval < 0)
  {
    logError("SocketDisable failed cellularConnectFlow");
  }

  return retval;
}

int cellularConnectFlow()
{
  logInfo("cellularConnectFlow");
  int result = -1;

  result = CellularInit();
  if (result < 0) {
    logError("cellularConnectFlow failed CellularInit, aborting.");
    return result;
  }

  logInfo("cellularConnectFlow validation with CellularCheckModem");
  result = CellularCheckModem();
  if (result < 0) {
    logError("cellularConnectFlow failed CellularCheckModem, aborting.");
    return result;
  }

  if (registerToNetwork)
  {
    logInfo("registerToNetwork=true");
    result = registeringToNetworkFlow();
    registerToNetwork = false;
    if (result < 0) {
      logError("cellularConnectFlow failed registeringToNetworkFlow, aborting.");
      return result;
    }
    logInfo("CellularRegistered!");
  }

  logInfo("cellularConnectFlow success");
  return result;
}

int registeringToNetworkFlow()
{
  logInfo("registeringToNetworkFlow");
  logDebug("allocating opList");
  OPERATOR_INFO opList[MAX_NUM_OPERATORS];
  memset(&opList, 0, sizeof(OPERATOR_INFO)*MAX_NUM_OPERATORS);
  if (opList == NULL) {
    logError("Memory allocation error for operator list, aborting.");
    return -1;
  }

  int numOpsFound = 0;
  if (detectCellularNetworks(opList, &numOpsFound)) {
    logError("Failed detectCellularNetworks");
    return -1;
  }

  logInfo("registerCellularNetwork");
  if (registerCellularNetwork(opList, numOpsFound)) {
    logError("Failed registerCellularNetwork");
    return -1;
  }

  registerToNetwork = false;
  return 0;
}

int detectCellularNetworks(OPERATOR_INFO* opList, int *numOpsFound) {
  logInfo("detectCellularNetworks");
  *numOpsFound = 0;
  int result = CellularGetOperators(opList, MAX_NUM_OPERATORS, numOpsFound);
  if (result < 0)
  {
    logError("detectCellularNetworks failed");
    return result;
  }
  if (*numOpsFound == 0)
  {
    logError("detectCellularNetworks failed - no operators");
    return -1;
  }

  logInfo("\tFound %d cellular networks!", *numOpsFound);
  return 0;
}

int registerCellularNetwork(OPERATOR_INFO* opList, int numOpsFound)
{
  logInfo("registerCellularNetwork");
  int result = 0;
  int modeForce = 1;
  int modeDisconnect = 2;
  int registrationStatus = 0;
  int isRegistered = 0;
  logInfo("CellularSetOperator(modeDisconnect, NULL)");
  if (CellularSetOperator(modeDisconnect, NULL) < 0)
  {
    logError("detectCellularNetworks failed at modeDisconnect");
    return -1;
  }

  OPERATOR_INFO selectedOp = {0};
  for (int i=0; i < numOpsFound; i++)
  {
    logInfo("CellularSetOperator attempt %d/%d", i+1, numOpsFound);
    logInfo("network name=%s;code=%d;ACT=%s;reg status=%d", 
            opList[i].operatorName, opList[i].operatorCode, opList[i].accessTechnology, registrationStatus);
    result = CellularSetOperator(modeForce, &opList[i]);
    if (result == 0)
    {
      for (int j=0; j < 3; j++)
      {
        if (CellularGetRegistrationStatus(&registrationStatus) < 0)
        {
          logError("Failed CellularGetRegistrationStatus");
          return -1;
        }

        if (registrationStatus == 1 || registrationStatus == 5)
        {
          logDebug("CellularSetOperator success, reg status=%d", registrationStatus);
          isRegistered = 1;
          selectedOp = opList[i];
          break;
        }

        logWarn("fail AT+CREG=%d, retrying in 5sec", registrationStatus);
        sleepMs(5000); // sleep for 5 seconds to see if AT+CREG? status is valid
      }

      if (isRegistered)
      {
        break;
      }
    }

    logWarn("CellularGetRegistrationStatus fail, disconnecting");
    if (CellularSetOperator(modeDisconnect, NULL)) {
      logError("detectCellularNetworks failed at modeDisconnect");
      return -1;
    }
  }

  if (!isRegistered) {
    logError("registerCellularNetwork fail");
    return -1;
  }

  logInfo("registerCellularNetwork Success!");
  logDebug("Network: '%s' (code=%d,ACT='%s',reg status=%d)", 
          selectedOp.operatorName, selectedOp.operatorCode, selectedOp.accessTechnology, registrationStatus);
  return 0;
}
