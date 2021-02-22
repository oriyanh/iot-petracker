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
#include "gps.h"


static byte MQTT_TX_BUFFER[BUFFER_SIZE] = {0};
static byte MQTT_RX_BUFFER[BUFFER_SIZE] = {0};
static char MQTT_PAYLOAD_BUFFER[MQTT_PAYLOAD_SIZE] = {0};

static volatile bool distress = false;
static volatile bool connected = false;

static DISPLAY_Device_t displayDevice;    /* Display device handle.         */

/**
 * Setup for on-device display
 */
static void DisplaySetUp(void);

/**
 * Runs main flow with MQTT - Init, Connect, Publish, DeInit
 */
static int openConnections(MQTTCtx* mqttCtx);

/**
 * Gets called when returning from `MqttFlow`, makes sure MQTT is de-initialized and 
 */
static int closeConnections(int retval, char *message, MQTTCtx *mqttCtx);

/**
 * Initializes MQTTCtx struct and MqttClient
 * @param mqttCtx Context
 */
static int initMqttCtx(MQTTCtx* mqttCtx);

/**
 * PUBLISHes current device location to an MQTT broker
 * @param mqttCtx Context
 * @return
 */
static int publishLocation(MQTTCtx* mqttCtx);

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

  setupTimer();
  /* Setup on-device display*/
  DisplaySetUp();

  MQTTCtx mqttCtx;
  int retval = initMqttCtx(&mqttCtx);
  if (retval < 0)
  {
    closeConnections(retval, "initMqttClient Error", &mqttCtx);
    while(1);
  }

  while (1)
  {
    // Connect to peripherals and MQTT broker
    if (!connected)
    {
      retval = openConnections(&mqttCtx);
      if (retval < 0)
      {
        closeConnections(retval, "connectFlow error", &mqttCtx);
      }
      else
      {
        connected = true;
      }
    }

    // Publish location if connected
    if (connected)
    {
      retval = publishLocation(&mqttCtx);
      if (retval < 0)
      {
        closeConnections(retval, "publish Error", &mqttCtx);
        connected = false;
      }
    }

    // Go to sleep if not in distress mode and disconnect
    if (!distress)
    {
      if (connected)
      {
        closeConnections(retval, "going to sleep", &mqttCtx);
        connected = false;
      }
      sleepMs(60000);
    }
    else
    {
      sleepMs(5000);
    }
  }
}

/**
 * Connects to broker
 * @param mqttCtx Context
 * @return
 */
static int connectToBroker(MQTTCtx* mqttCtx);

/**
 * Subscribes to Distress topic
 * @param mqttCtx Context
 * @return
 */
static int subscribeToTopic(MQTTCtx* mqttCtx);

int openConnections(MQTTCtx* mqttCtx)
{
  logInfo("openConnections");
  int retval = GPSInit();
  if (retval < 0)
  {
    logError("openConnections failed GPSInit");
    return retval;
  }

  logInfo("connectToBroker: %s:%d", MQTT_DASHBOARD, MQTT_PORT);
  retval = connectToBroker(mqttCtx);
  if (retval < 0)
  {
    return closeConnections(retval, "connectToBroker Error", mqttCtx);
  }

  retval = subscribeToTopic(mqttCtx);
  if (retval < 0)
  {
    return closeConnections(retval, "subscribeToTopic Error", mqttCtx);
  }

  return MQTT_CODE_SUCCESS;
}

static int closeConnections(int retval, char *message, MQTTCtx *mqttCtx)
{
  if (retval < 0)
  {
    logFail("Error: %s\nError code (%d): %s\n rebooting", message, retval, MqttClient_ReturnCodeToString(retval));
    mqttCtx->reboot = 1;
    sleepMs(5000);
  }

  else
  {
    logSuccess("success, %s", message);
    mqttCtx->reboot = 0;
  }

  logInfo("closing external connections");
  GPSDisable();
  MqttClientNet_DeInit(&mqttCtx->net);
  return retval;
}

/**
 * Callback that is executed whenever a PUBLISH message is received in a topic the client is subscribed to
 * @param client MQTT client
 * @param message Incoming message
 * @param msg_new Whether the message is new or not
 * @param msg_done
 * @return
 */
static int DistressMqttCallback(MqttClient *client, MqttMessage *message, byte msg_new, byte msg_done)
{
  logInfo("DistressMqttCallback");
  byte buf[BUFFER_SIZE + 1];
  word32 len;

  if (msg_new)
  {
    /* Determine min size to dump */
    len = message->topic_name_len;
    if (len > BUFFER_SIZE)
    {
      len = BUFFER_SIZE;
    }
    XMEMCPY(buf, message->topic_name, len);
    buf[len] = '\0'; /* Make sure its null terminated */
    if (strstr((char*)buf, DISTRESS_ENABLE) != NULL)
    {
      logInfo("distress enabled");
      distress = true;
    }
    else if (strstr((char*)buf, DISTRESS_DISABLE) != NULL)
    {
      logInfo("distress disabled");
      distress = false;
    }
  }

  len = message->buffer_len;
  if (len > BUFFER_SIZE)
  {
    len = BUFFER_SIZE;
  }
  memcpy(buf, message->buffer, len);
  buf[len] = '\0'; /* Make sure its null terminated */
  if (strstr((char*)buf, DISTRESS_ENABLE) != NULL)
  {
    logInfo("distress enabled");
    distress = true;
  }
  else if (strstr((char*)buf, DISTRESS_DISABLE) != NULL)
  {
    logInfo("distress disabled");
    distress = false;
  }

  /* Return negative to terminate publish processing */
  return MQTT_CODE_SUCCESS;
}

static int initMqttCtx(MQTTCtx* mqttCtx)
{
  logInfo("initMqttCtx");
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
  
  logInfo("InitMqttClient");
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
  mqttCtx->client.msg_cb = DistressMqttCallback;
  mqttCtx->reboot = 0;

  logInfo("MqttClientNet_Init");
  return MqttClientNet_Init(mqttCtx->client.net, mqttCtx);
}

/**
 * Initializes CONNECT packet
 * @param mqttCtx Context
 */
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
  if (mqttCtx->enable_lwt)
  {
    /* Send client id in LWT payload */
    mqttCtx->lwt_msg.qos = mqttCtx->qos;
    mqttCtx->lwt_msg.retain = 0;
    mqttCtx->lwt_msg.topic_name = mqttCtx->topic_name;
    mqttCtx->lwt_msg.buffer = (byte*)mqttCtx->client_id;
    mqttCtx->lwt_msg.total_len = (word16)XSTRLEN(mqttCtx->client_id);
  }
}

static int connectToBroker(MQTTCtx* mqttCtx)
{
  logInfo("connectToBroker");
  int retval = MqttClient_NetConnect(&(mqttCtx->client), mqttCtx->host, mqttCtx->port, mqttCtx->cmd_timeout_ms, 0,NULL);
  if (retval < 0)
  {
    logError("MqttClient_NetConnect fail");
    return retval;
  }

  initConnectPacket(mqttCtx);
  retval = MqttClient_Connect(&(mqttCtx->client), &(mqttCtx->connect));
  if (retval < 0)
  {
    logError("MqttClient_Connect fail");
  }
  return retval;
}

/**
 * Prepares payload for `publishMessage`
 */
static int preparePayload(char* buff)
{
  static const char* PAYLOAD_FORMAT = "%*.c";
  GPS_LOCATION_INFO location = {0};
  memset(&location, 0, sizeof(GPS_LOCATION_INFO));
  if (!GPSGetFixInformation(&location))
  {
    logError("preparePayload fail");
    return -1;
  }
  char payload[sizeof(GPS_LOCATION_INFO)+1] = {0};
  sprintf(payload, PAYLOAD_FORMAT, sizeof(GPS_LOCATION_INFO), '0');
  memcpy(payload, &location, sizeof(GPS_LOCATION_INFO));
  memcpy(buff, payload, sizeof(payload));
  return sizeof(GPS_LOCATION_INFO);
}

static int publishLocation(MQTTCtx* mqttCtx)
{
  logInfo("publishLocation");
  MqttMessage publish;
  XMEMSET(&publish, 0, sizeof(MqttPublish));
  publish.stat = MQTT_MSG_BEGIN;
  publish.retain = 1;
  publish.qos = mqttCtx->qos;
  publish.duplicate = 0;
  publish.topic_name = LOCATION_TOPIC;
  publish.packet_id = 2;
  int payloadLen = preparePayload(MQTT_PAYLOAD_BUFFER);
  if (payloadLen < 0)
  {
    logError("publishLocation fail");
    return payloadLen;
  }
  publish.buffer = (byte*)MQTT_PAYLOAD_BUFFER;
  publish.total_len = (word16)payloadLen;
  logInfo("Publish to topic '%s'", publish.topic_name);
  return MqttClient_Publish(&(mqttCtx->client), &publish);
}

static int subscribeToTopic(MQTTCtx* mqttCtx)
{
  logInfo("SubscribeToTopic %s", mqttCtx->topic_name);
  XMEMSET(&(mqttCtx->subscribe), 0, sizeof(MqttSubscribe));
  mqttCtx->topics[0].topic_filter = DISTRESS_TOPIC;
  mqttCtx->topics[0].qos = mqttCtx->qos;
  mqttCtx->subscribe.packet_id = 1;
  mqttCtx->subscribe.topic_count =
      sizeof(mqttCtx->topics) / sizeof(MqttTopic);
  mqttCtx->subscribe.topics = mqttCtx->topics;
  int retval = MqttClient_Subscribe(&(mqttCtx->client), &(mqttCtx->subscribe));
  if (retval < 0)
  {
    return retval;
  }
  return retval;
}

static void DisplaySetUp(void)
{
  DISPLAY_Init();

  /* Retrieve the properties of the display. */
  if (DISPLAY_DeviceGet(0, &displayDevice) != DISPLAY_EMSTATUS_OK)
  {
    logError("Unable to get display handle");
    while (1) ;
  }

  /* Retarget stdio to the display. */
  if (TEXTDISPLAY_EMSTATUS_OK != RETARGET_TextDisplayInit())
  {
    logError("Text display initialization failed");
    while (1) ;
  }
}
