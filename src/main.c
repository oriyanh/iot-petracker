/***************************************************************************//**
 * @file
 * @brief Energy Mode demo for SLSTK3402A
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "display.h"
#include "textdisplay.h"
#include "retargettextdisplay.h"
#include "bsp.h"

#include "wolfmqtt/mqtt_types.h"
#include "wolfmqtt/mqtt_socket.h"
#include "wolfmqtt/mqtt_packet.h"

#include "config.h"
#include "logger.h"
#include "timer_manager.h"
#include "cellular.h"
#include "MQTTclient.h"



/**
 * Setup PB0 and PB1 interrupts
 */
static void GpioSetup(void);

/**
 * Enable PB0 and PB1 interrupts
 */
static void GpioEnable();

/**
 * Disable PB0 and PB1 interrupts
 */
static void GpioDisable();

/**
 * Setup for on-device display
 */
static void DisplaySetUp(void);
/**
 * Redraws display
 */
static void drawScreen(void);


/**
 * Runs main flow with MQTT - Init, Connect, Publish, DeInit
 */
static int MqttConnectFlow(MQTTCtx* mqttCtx);

/**
 * Initializes MqttCtx struct
 * @param mqttCtx Context
 */
static void initMqttCtx(MQTTCtx* mqttCtx);

/**
 * Initializes MqttClient struct of MqttCtx
 * @param mqttCtx Context
 * @return
 */
static int initMqttClient(MQTTCtx* mqttCtx);

/**
 * Initializes CONNECT packet
 * @param mqttCtx Context
 */
static void initConnectPacket(MQTTCtx *mqttCtx);

/**
 * Connects to broker
 * @param mqttCtx Context
 * @return
 */
static int connectToBroker(MQTTCtx* mqttCtx);

/**
 * PUBLISHes a message
 * @param mqttCtx Context
 * @return
 */
static int publishMessage(MQTTCtx* mqttCtx);

static int subscribeToTopic(MQTTCtx* mqttCtx);

/**
 * Prepares payload for `publishMessage`
 */
static void preparePayload(char* buff, size_t len);

/**
 * Gets called when returning from `MqttFlow`, makes sure MQTT is de-initialized and 
 */
static int MqttExitRoutine(int retval, char *message, MQTTCtx *mqttCtx);

/**
 * Callback that is executed whenever a PUBLISH message is received in a topic the client is subscribed to
 * @param client MQTT client
 * @param message Incoming message
 * @param msg_new Whether the message is new or not
 * @param msg_done
 * @return
 */
static int DitressListener(MqttClient *client, MqttMessage *message, byte msg_new, byte msg_done);


static byte MQTT_TX_BUFFER[BUFFER_SIZE] = {0};
static byte MQTT_RX_BUFFER[BUFFER_SIZE] = {0};
static char MQTT_PAYLOAD_BUFFER[PAYLOAD_SIZE] = {0};

static volatile bool redrawScreen = false; /* Draw screen again or not*/
static volatile bool distress = false;
static volatile bool connected = false;

static DISPLAY_Device_t displayDevice;    /* Display device handle.         */
int volatile msgCount=0; //TODO remove - this is for debug purposes



/***************************************************************************//**
 * @brief  Main function
 ******************************************************************************/
int main(void)
{
  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_STK_DEFAULT;
  CMU_HFXOInit_TypeDef hfxoInit = CMU_HFXOINIT_STK_DEFAULT;

  /* Chip errata */
  CHIP_Init();

  /* Init DCDC regulator and HFXO with kit specific parameters */
  EMU_DCDCInit(&dcdcInit);
  CMU_HFXOInit(&hfxoInit);


  /* Switch HFCLK to HFXO and disable HFRCO */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
  CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

  /* Setup GPIO for pushbuttons. */
  GpioSetup();
  GpioEnable();

  setupTimer();
  /* Setup on-device display*/
  DisplaySetUp();

  /* Triggers initial screen draw*/
  redrawScreen = true;
// TODO do connect + registeration to network.
  MQTTCtx mqttCtx;

  initMqttCtx(&mqttCtx);
  int retval = 0;
  logInfo("initMqttClient");
  retval = initMqttClient(&mqttCtx);
  if (retval < 0) {
	  return MqttExitRoutine(retval, "initMqttClient Error", &mqttCtx);
  }

  while (1) {

	  if (!connected){
		  retval = MqttConnectFlow(&mqttCtx);
		  if (retval<0){
			  return -1;
		  }
		  connected = true;
	  }
	  retval = publishMessage(&mqttCtx);
	  if (retval < 0){
		  return MqttExitRoutine(retval, "publish Error", &mqttCtx);
	  }

	  if (!distress){
		  MqttClientNet_DeInit(&mqttCtx.net);
		  connected = false;
		  sleepMs(60000);
	  }
	  else{
		  sleepMs(10000);
	  }

	  if (redrawScreen) {
		  drawScreen();
	  }
  }
}

int MqttConnectFlow(MQTTCtx* mqttCtx){
  logInfo("start MqttFlow");
  int retval=0;
  logInfo("broker: %s:%d", MQTT_DASHBOARD, MQTT_PORT);

  logInfo("connectToBroker");
  retval = connectToBroker(mqttCtx);
  if (retval < 0) {
	  return MqttExitRoutine(retval, "connectToBroker Error", &mqttCtx);
  }

  logInfo("subscribeMessage");
  retval = subscribeToTopic(mqttCtx);
  if (retval < 0){
      return MqttExitRoutine(retval, "publishMessage Error", &mqttCtx);
  }

  return MQTT_CODE_SUCCESS;
}

static void initMqttCtx(MQTTCtx* mqttCtx) {
  XMEMSET(mqttCtx, 0, sizeof(MQTTCtx));
  mqttCtx->host = MQTT_DASHBOARD;
  mqttCtx->port = MQTT_PORT;
  mqttCtx->qos = MQTT_QOS_1;
  mqttCtx->clean_session = 0;
  mqttCtx->keep_alive_sec = DEFAULT_KEEP_ALIVE_SEC;
  mqttCtx->client_id = "MayOrOfFlavortown";
  mqttCtx->topic_name = DISTRESS_TOPIC;
  mqttCtx->cmd_timeout_ms = MQTT_CMD_TIMEOUT_MS;
  mqttCtx->retain = 0;
  mqttCtx->app_name = "MayOr (mayonnaise-filled Oreo)";
  mqttCtx->message = NULL;
  mqttCtx->tx_buf = MQTT_TX_BUFFER;
  mqttCtx->rx_buf = MQTT_RX_BUFFER;
  MqttNet network;
  XMEMSET(&network, 0, sizeof(MqttNet));
  mqttCtx->net = network;
}

static int initMqttClient(MQTTCtx* mqttCtx) {
  MqttClient client;
  memset(&client, 0, sizeof(MqttClient));
  mqttCtx->client.ctx = &mqttCtx;
  mqttCtx->client = client;
  mqttCtx->client.cmd_timeout_ms = MQTT_CMD_TIMEOUT_MS;
  mqttCtx->client.net = &(mqttCtx->net);
  mqttCtx->client.tx_buf = MQTT_TX_BUFFER;
  mqttCtx->client.rx_buf = MQTT_RX_BUFFER;
  mqttCtx->client.tx_buf_len = BUFFER_SIZE;
  mqttCtx->client.rx_buf_len = BUFFER_SIZE;
  mqttCtx->client.msg_cb = DitressListener;
  mqttCtx->reboot = 0;

  logInfo("MqttClientNet_Init");
  return MqttClientNet_Init(mqttCtx->client.net, mqttCtx);
}

/**
 * Callback that is executed whenever a PUBLISH message is received in a topic the client is subscribed to
 * @param client MQTT client
 * @param message Incoming message
 * @param msg_new Whether the message is new or not
 * @param msg_done
 * @return
 */
static int DitressListener(MqttClient *client, MqttMessage *message,
                     byte msg_new, byte msg_done) {
    logInfo("There is a message call back!");
    byte buf[BUFFER_SIZE + 1];
    word32 len;
    MQTTCtx* mqttCtx = (MQTTCtx*) client->ctx;

    (void)mqttCtx;

    if (msg_new) {
        /* Determine min size to dump */
        len = message->topic_name_len;
        if (len > BUFFER_SIZE) {
            len = BUFFER_SIZE;
        }
        XMEMCPY(buf, message->topic_name, len);
        buf[len] = '\0'; /* Make sure its null terminated */

        /* Print incoming message */
        logInfo("MQTT Message: Topic %s, Qos %d, Len %u",
               buf, message->qos, message->total_len);

    }

    len = message->buffer_len;
    if (len > BUFFER_SIZE) {
        len = BUFFER_SIZE;
    }
    XMEMCPY(buf, message->buffer, len);
    buf[len] = '\0'; /* Make sure its null terminated */
    if (strstr(buf, DISTRESS_ENABLE)== NULL){
    	logInfo("distress enabled");
    	distress = true;
    	redrawScreen =true;
    }
    else if (strstr(buf, DISTRESS_DISABLE)==NULL){
    	logInfo("distress disabled");
    	distress = false;
    	redrawScreen =true;
	}
    logInfo("Payload (%d - %d): %s",
               message->buffer_pos, message->buffer_pos + len, buf);
    if (msg_done) {
        PRINTF("MQTT Message: Done");
    }

    /* Return negative to terminate publish processing */
    return MQTT_CODE_SUCCESS;
}



static void initConnectPacket(MQTTCtx *mqttCtx)
{
  /* Build connect packet */
  MqttConnect connect;
  XMEMSET(&connect, 0, sizeof(MqttConnect));
  mqttCtx->connect = connect;
  mqttCtx->connect.stat = MQTT_MSG_BEGIN;
  XMEMSET(&mqttCtx->connect, 0, sizeof(MqttConnect));
  mqttCtx->connect.keep_alive_sec = mqttCtx->keep_alive_sec;
  mqttCtx->connect.clean_session = mqttCtx->clean_session;
  mqttCtx->connect.client_id = mqttCtx->client_id;

  XMEMSET(&mqttCtx->lwt_msg, 0, sizeof(mqttCtx->lwt_msg));
  mqttCtx->connect.lwt_msg = &mqttCtx->lwt_msg;
  mqttCtx->connect.enable_lwt = mqttCtx->enable_lwt;
  if (mqttCtx->enable_lwt) {
      /* Send client id in LWT payload */
      mqttCtx->lwt_msg.qos = mqttCtx->qos;
      mqttCtx->lwt_msg.retain = 0;
      mqttCtx->lwt_msg.topic_name = mqttCtx->topic_name;
      mqttCtx->lwt_msg.buffer = (byte*)mqttCtx->client_id;
      mqttCtx->lwt_msg.total_len = (word16)XSTRLEN(mqttCtx->client_id);
  }
}

static int connectToBroker(MQTTCtx* mqttCtx) {
  logDebug("connectToBroker");
  int retval = MqttClient_NetConnect(&(mqttCtx->client), mqttCtx->host, mqttCtx->port, mqttCtx->cmd_timeout_ms, 0,NULL);
  if (retval < 0) {
      return retval;
  }
  initConnectPacket(mqttCtx);
  retval = MqttClient_Connect(&(mqttCtx->client), &(mqttCtx->connect));
  return retval;
}

static void preparePayload(char* buff, size_t len){
	return;
//  memset(buff, 0, len);
//  MODEM_METADATA metaData = {0};
//  GetModemMetadata(&metaData);
//  snprintf(buff, len, PAYLOAD_FORMAT, metaData.ccid, metaData.op_info.operatorName,
//	   metaData.op_info.operatorCode,metaData.op_info.accessTechnology,metaData.csq, pb1Clicks);
}

static int publishMessage(MQTTCtx* mqttCtx){
  MqttMessage publish;
  XMEMSET(&publish, 0, sizeof(MqttPublish));
  publish.stat = MQTT_MSG_BEGIN;
  publish.retain = 1;
  publish.qos = mqttCtx->qos;
  publish.duplicate = 0;
  publish.topic_name = LOCATION_TOPIC;
  publish.packet_id = 2;
  sprintf(MQTT_PAYLOAD_BUFFER, FAKE_PAYLOAD, msgCount++);
  publish.buffer = (byte*)MQTT_PAYLOAD_BUFFER;

  logInfo("Publish to topic '%s'", publish.topic_name, publish.buffer);
  logInfo("Payload:\n%s", publish.topic_name, publish.buffer);
  publish.total_len = (word16)XSTRLEN((char*)publish.buffer);
  return MqttClient_Publish(&(mqttCtx->client), &publish);
}

static int subscribeToTopic(MQTTCtx* mqttCtx){
	printf("3) Subscribe to topic %s\n", mqttCtx->topic_name);
	// Build list of topics
	XMEMSET(&(mqttCtx->subscribe), 0, sizeof(MqttSubscribe));
	mqttCtx->topics[0].topic_filter = DISTRESS_TOPIC;
	mqttCtx->topics[0].qos = mqttCtx->qos;
	mqttCtx->subscribe.packet_id = 1;
	mqttCtx->subscribe.topic_count =
			sizeof(mqttCtx->topics) / sizeof(MqttTopic);
	mqttCtx->subscribe.topics = mqttCtx->topics;
	int retval = MqttClient_Subscribe(&(mqttCtx->client), &(mqttCtx->subscribe));
	if (retval < 0) {
		return retval;
	}
	return retval;
}




static int MqttExitRoutine(int retval, char *message, MQTTCtx *mqttCtx) {
  if (retval < 0){
      logError("%s\nError code (%d): %s", message, retval, MqttClient_ReturnCodeToString(retval));
//      logFail("MqttFlow failure, aborting...");
      sleepMs(5000);
      mqttCtx->reboot = 1;
  }

  else {
	  mqttCtx->reboot = 0;
      logSuccess("%s", message);
  }

  logInfo("MqttClientNet_DeInit");
  MqttClientNet_DeInit(&mqttCtx->net);
  return retval;
}


static void drawScreen(void) {
  redrawScreen = false;
//  printf("\f");
//  if(applicationMode == MENU_DISPLAY_MODE){
//      printf("Menu:\n\n"
//	  "PB0: Publish message to MQTT broker. \n"
//	  "PB1: Number of clicks since last sent information\n\n");
//
//  }
//  else if(applicationMode == CLICKS_DISPLAY_MODE){
////      printf("# PB1 presses: %lu\n", pb1Clicks);
//  }
  if(distress){
	  printf("distress");
  }
}

static void GpioSetup(void)
{
  /* Enable GPIO clock. */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure PB0 as input and enable interrupt  */
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInputPull, 1);
  GPIO_IntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, false, true, true);

  /* Configure PB1 as input and enable interrupt */
  GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInputPull, 1);
  GPIO_IntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, false, true, true);

  GpioEnable();
}

static void GpioEnable(){
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

static void GpioDisable(){
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_DisableIRQ(GPIO_EVEN_IRQn);

  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_DisableIRQ(GPIO_ODD_IRQn);
}

static void DisplaySetUp(void){
  DISPLAY_Init();

  /* Retrieve the properties of the display. */
  if (DISPLAY_DeviceGet(0, &displayDevice) != DISPLAY_EMSTATUS_OK) {
      /* Unable to get display handle. */
      while (1) ;
  }

  /* Retarget stdio to the display. */
  if (TEXTDISPLAY_EMSTATUS_OK != RETARGET_TextDisplayInit()) {
      /* Text display initialization failed. */
      while (1) ;
  }
}


/***************************************************************************//**
 * @brief Unified GPIO Interrupt handler (pushbuttons)
 *        PB0 Starts selected test
 *        PB1 Cycles through the available tests
 ******************************************************************************/
void GPIO_Unified_IRQ(void)
{
  /* Get and clear all pending GPIO interrupts */
  uint32_t interruptMask = GPIO_IntGet();
  GPIO_IntClear(interruptMask);

  /* Act on interrupts */
  if (interruptMask & (1 << BSP_GPIO_PB1_PIN)) {
      distress = true;
//	  applicationMode = CLICKS_DISPLAY_MODE;
//      redrawScreen = true;
  }
  else if (interruptMask & (1 << BSP_GPIO_PB0_PIN)) {
	  distress = false;
//	  redrawScreen = true;
//      applicationMode = SEND_UART_MODE;
  }

}

/***************************************************************************//**
 * @brief GPIO Interrupt handler for even pins
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  GPIO_Unified_IRQ();
}

/***************************************************************************//**
 * @brief GPIO Interrupt handler for odd pins
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  GPIO_Unified_IRQ();
}
