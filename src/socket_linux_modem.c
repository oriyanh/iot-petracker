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

#define MAX_NUM_OPS 10


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
 * Runs cellularInit->CheckModem->detectNetworks->register->refresh. This function is called in SocketInit
 */
static int cellularConnectFlow(void);

int SocketInit(const char *host, word16 port)
{

    if (cellularConnectFlow() < 0) {
        logError("Failed to open socket, aborting");
        return -1;
    }

    logInfo("SocketInit(%s:%d)", host, port);

    if (CellularSetupInternetConnectionProfile(20) < 0)
    {
	logError("CellularSetupInternetConnectionProfile failed");
	return -1;
    }


    if (CellularSetupInternetServiceSetupProfile(host, (unsigned int)port, 200) < 0)
    {
    	logError("CellularSetupInternetServiceSetupProfile failed");
    	return -1;
    }

    logInfo("SocketInit success");
    return 0;
}

int SocketConnect(void)
{
    logInfo("SocketConnect");
    if (CellularConnect() < 0)
    {
        logError("Failed to create TCP connection. Reason:");
        CellularClose();
        return -1;
    }

    logInfo("SocketConnect success");

    return 0;
}

int SocketWrite(const unsigned char *payload, unsigned int len)
{
    logInfo("SocketWrite %d bytes", len);

    logDebug("Writing...");
    int numBytesSent = CellularWrite(payload , len);
    if( numBytesSent < 0)
    {
        logError("Failed SocketWrite. Reason");
        return -1;
    }

    logDebug("Successfully wrote %d bytes to socket", numBytesSent);
    return numBytesSent;
}

int SocketRead(unsigned char *buf, unsigned int max_len, unsigned int timeout_ms) {
    logInfo("SocketRead %u bytes,timeout %ums", max_len, timeout_ms);
    int numBytesReceived = CellularRead(buf,max_len,timeout_ms);
    if(numBytesReceived < 0)
    {
        logError("Failed SocketRead. Reason");
        return -1;
    }

    logInfo("SocketRead success", numBytesReceived);

    return (int) numBytesReceived;
}

int SocketClose() {
    logInfo("SocketClose");

    return CellularClose();
}
int SocketDisable() {
	logInfo("SocketDisable");
	return CellularDisable();
}


int cellularConnectFlow() {
    logInfo("cellularConnectFlow");
    int result;

    result = CellularInit(MODEM_PORT);
    if (result < 0) {
        logError("Failed CellularInit, aborting.");
        CellularDisable();
        return -1;
    }
    logInfo("CellularInit success!");

    logInfo("Validating cellularConnectFlow");
    result = CellularCheckModem();
    if (result < 0) {
        logError("Failed to validate connectivity, aborting.");
        CellularDisable();
        return -1;
    }
//    logInfo("CellularCheckModem validated!");
//    if (registerToNetwork) {
//		logInfo("detectCellularNetworks");
//		logDebug("allocating opList");
//		OPERATOR_INFO* opList = NULL;
//		opList = (OPERATOR_INFO*) calloc(MAX_NUM_OPS, sizeof(OPERATOR_INFO));
//		if (opList == NULL) {
//			logError("Memory allocation error for operator list, aborting.");
//			CellularDisable();
//			return -1;
//		}
//
//		int numOpsFound = 0;
//		if (detectCellularNetworks(opList, &numOpsFound)) {
//			logError("Failed detectCellularNetworks");
//			free(opList);
//			opList = NULL;
//			CellularDisable();
//			return -1;
//		}
//
//		logInfo("registerCellularNetwork");
//		if (registerCellularNetwork(opList, numOpsFound)) {
//			logError("Failed registerCellularNetwork");
//			free(opList);
//			opList = NULL;
//			CellularDisable();
//			return -1;
//		}
//
//		free(opList);
//		opList = NULL;
//		registerToNetwork = false;
//	}
    return 0;
}


int detectCellularNetworks(OPERATOR_INFO* opList, int *numOpsFound) {
    logDebug("querying modem for cellular operators");
    int result = CellularGetOperators(opList, MAX_NUM_OPS, numOpsFound);
    if (result == 0) {
        logInfo("\tFound %d cellular networks!", *numOpsFound);
    }
    else if (numOpsFound == 0) {
        logError("Did not find any operators");
        free(opList);
        return -1;
    }
    else {
        logError("Got some sort of error");
        perror("System error is (if occurred):");
        free(opList);
        return -1;
    }
    return 0;
}

int registerCellularNetwork(OPERATOR_INFO* opList, int numOpsFound) {
    int result = 0;
    int modeForce = 1;
    int modeDisconnect = 2;
    int registrationStatus = 0;
    int isRegistered = 0;
    logInfo("CellularSetOperator(modeDisconnect, NULL)");
    if (CellularSetOperator(modeDisconnect, NULL)) {
        logError("Failed at disconnecting from cellular network, exiting program");
        return -1;
    }
    OPERATOR_INFO selectedOp = {0};
    for (int i=0; i < numOpsFound; i++) {
        logInfo("\tAttempt %d / %d to register to a cellular network:", i+1, numOpsFound);
        logInfo("\t\tnetwork name: '%s', network code %d, ACT '%s', registration status: %d", 
                    opList[i].operatorName, opList[i].operatorCode, opList[i].accessTechnology, registrationStatus);
        result = CellularSetOperator(modeForce, &opList[i]);
        if (result == 0) {
            for (int j=0; j < 3; j++)
	    {
	      if (CellularGetRegistrationStatus(&registrationStatus) < 0) {
		  logError("Failed CellularGetRegistrationStatus");
		  return -1;
	      }
	      if (registrationStatus == 1 || registrationStatus == 5) {
		  logDebug("\tSuccessfully registered to network with registration status: %d", registrationStatus);
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

        logWarn("\tCould not register, disconnecting");
        if (CellularSetOperator(modeDisconnect, NULL)) {
            logError("Failed at disconnecting from cellular network, exiting program");
            return -1;
        }
    }

    if (!isRegistered) {
        logError("Failed to register to a cellular network, exiting program");
        return -1;
    }

    logInfo("registerCellularNetwork Success! '%s' (code=%d,ACT='%s',reg status=%d)", 
                    selectedOp.operatorName, selectedOp.operatorCode, selectedOp.accessTechnology, registrationStatus);
    return 0;
}

