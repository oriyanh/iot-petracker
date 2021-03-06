#include "wolfmqtt/mqtt_types.h"
#include "MQTTClient.h"
#include "socket.h"
#include "config.h"
#include "logger.h"

/**
 * Connects the socket to the broker, to the given `host` and `port`.
 * Usage of `context` is optional, although recommended. Returns 0 on success, and a negative number otherwise
 * (one of `MqttPacketResponseCodes`). `timeout_ms` defines the timeout in milliseconds.
 * @param context Client context, UNUSUED
 * @param host
 * @param port
 * @param timeout_ms UNUSED
 * @return
 */
static int NetConnect(void *context, const char* host, word16 port, int timeout_ms)
{
  int retval;
  retval = SocketInit(host, port);
  if (retval < 0)
  {
    logError("NetConnect fail at SocketInit");
    return retval;    
  }

  logInfo("SocketConnect");
  retval = SocketConnect();
  if (retval < 0)
  {
    logError("NetConnect fail at SocketConnect");
    return retval;    
  }

  logInfo("NetConnect success");
  return MQTT_CODE_SUCCESS;
}

/**
 * Performs a network (socket) read from the connected broker, to the given buffer `buf`, and reads `buf_len` bytes.
 * Usage of `context` is optional, although recommended.
 * Returns number of read bytes on success, and a negative number otherwise (one of `MqttPacketResponseCodes`).
 * timeout_ms` defines the timeout in milliseconds.
 * @param context
 * @param buf
 * @param buf_len
 * @param timeout_ms
 * @return
 */
static int NetRead(void *context, byte* buf, int buf_len, int timeout_ms)
{
  logDebug("Reading data from MQTT broker");
  int numBytes = SocketRead(buf, buf_len, timeout_ms);
  return numBytes;
}


/**
 * Performs a network (socket) write to the connected broker, from the given buffer `buf`, and writes `buf_len` bytes.
 * Usage of `context` is optional, although recommended.
 * Returns number of written bytes on success, and a negative number otherwise (one of `MqttPacketResponseCodes`).
 * timeout_ms` defines the timeout in milliseconds.
 * @param context Client context, UNUSED
 * @param buf Write buffer
 * @param buf_len number of bytes to write
 * @param timeout_ms UNUSED
 * @return
 */
static int NetWrite(void *context, const byte* buf, int buf_len, int timeout_ms)
{
  logInfo("NetWrite(timeout %d ms, %d bytes)", timeout_ms, buf_len);
  int numBytes = SocketWrite(buf, buf_len);
  return numBytes;
}

/**
 * Disconnects from broker
 * @param context Client context, UNUSED
 * @return
 */
static int NetDisconnect(void *context)
{
  logDebug("NetDisconnect");
  int retval = MQTT_CODE_SUCCESS;
  if (context == NULL)
  {
    retval = SocketDisable();
  }
  else
  {
    MQTTCtx* mqttCtx = (MQTTCtx*) context;
    if (mqttCtx->reboot)
    {
      retval= SocketDisable();
    }

    else
    {
      retval = SocketClose();
    }
  }

  if (retval < 0)
  {
    logError("NetDisconnect error");
    return MQTT_CODE_ERROR_NETWORK;
  }

  return MQTT_CODE_SUCCESS;
}

/**
 * Initializes MqttNet struct with callbacks, and initializes the MQTT client data structures
 * @param net Network struct to initialize
 * @param mqttCtx Client context
 * @return
 */
int MqttClientNet_Init(MqttNet* net, MQTTCtx* mqttCtx)
{
  net->connect = NetConnect;
  net->read = NetRead;
  net->write = NetWrite;
  net->disconnect = NetDisconnect;
  net->context = mqttCtx;
  mqttCtx->client.net = net;
  int retval = MqttClient_Init(&(mqttCtx->client), mqttCtx->client.net, mqttCtx->client.msg_cb, mqttCtx->client.tx_buf, mqttCtx->client.tx_buf_len,
                               mqttCtx->client.rx_buf, mqttCtx->client.rx_buf_len, mqttCtx->cmd_timeout_ms);
  return retval;
}

/**
 * Disconnects from MQTT broker
 * @param net Network struct to deinitialize
 * @return
 */
int MqttClientNet_DeInit(MqttNet* net)
{
  logInfo("MqttClientNet_DeInit");
  MQTTCtx* mqttCtx = net->context;

  logInfo("MqttClient_Unsubscribe");
  XMEMSET(&mqttCtx->unsubscribe, 0, sizeof(MqttUnsubscribe));
  mqttCtx->unsubscribe.packet_id = 999;
  mqttCtx->unsubscribe.topic_count = 1;
  mqttCtx->unsubscribe.topics = mqttCtx->topics;

  /* Unsubscribe Topics */
  int retval = MqttClient_Unsubscribe(&mqttCtx->client,
                                      &mqttCtx->unsubscribe);

  if (retval < 0)
  {
    logError("MqttClient_Disconnect fail");
    return retval;      
  }

  retval = MqttClient_Disconnect(&(mqttCtx->client));
  if (retval < 0)
  {
    logError("MqttClient_Disconnect fail");
    return retval;      
  }
  retval = MqttClient_NetDisconnect(&(mqttCtx->client));
  if (retval < 0)
  {
    logError("MqttClient_NetDisconnect fail");
    return retval;      
  }
  MqttClient_DeInit(&(mqttCtx->client));
  return retval;
}
