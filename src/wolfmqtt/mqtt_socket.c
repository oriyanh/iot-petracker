/* mqtt_socket.c
 *
 * Copyright (C) 2006-2020 wolfSSL Inc.
 *
 * This file is part of wolfMQTT.
 *
 * wolfMQTT is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * wolfMQTT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA
 */

/* Include the autoconf generated config.h */
#ifdef HAVE_CONFIG_H
    #include <config.h>
#endif


#include "mqtt_client.h"
#include "mqtt_socket.h"

/* Options */
//#define WOLFMQTT_DEBUG_SOCKET
#ifdef WOLFMQTT_NO_STDIO
    #undef WOLFMQTT_DEBUG_SOCKET
#endif


/* Public Functions */

int MqttSocket_Init(MqttClient *client, MqttNet *net)
{
    int rc = MQTT_CODE_ERROR_BAD_ARG;
    if (client) {
        client->net = net;
        client->flags &= ~(MQTT_CLIENT_FLAG_IS_CONNECTED |
            MQTT_CLIENT_FLAG_IS_TLS);

        /* Validate callbacks are not null! */
        if (net && net->connect && net->read && net->write && net->disconnect) {
            rc = MQTT_CODE_SUCCESS;
        }
    }
    return rc;
}

static int MqttSocket_WriteDo(MqttClient *client, const byte* buf, int buf_len,
    int timeout_ms)
{
    int rc;

    {
        rc = client->net->write(client->net->context, buf, buf_len,
            timeout_ms);
    }

#ifdef WOLFMQTT_DEBUG_SOCKET
    if (rc != 0 && rc != MQTT_CODE_CONTINUE) { /* hide in non-blocking case */
        PRINTF("MqttSocket_Write: Len=%d, Rc=%d", buf_len, rc);
    }
#endif

    return rc;
}

int MqttSocket_Write(MqttClient *client, const byte* buf, int buf_len,
    int timeout_ms)
{
    int rc;

    /* Validate arguments */
    if (client == NULL || client->net == NULL || client->net->write == NULL ||
        buf == NULL || buf_len <= 0) {
        return MQTT_CODE_ERROR_BAD_ARG;
    }

    /* check for buffer position overflow */
    if (client->write.pos >= buf_len) {
        return MQTT_CODE_ERROR_OUT_OF_BUFFER;
    }

    do {
        rc = MqttSocket_WriteDo(client, &buf[client->write.pos],
            buf_len - client->write.pos, timeout_ms);
        if (rc <= 0) {
            break;
        }
        client->write.pos += rc;
    } while (client->write.pos < buf_len);

    /* handle return code */
    if (rc > 0) {
        /* return length write and reset position */
        rc = client->write.pos;
        client->write.pos = 0;
    }

    return rc;
}

static int MqttSocket_ReadDo(MqttClient *client, byte* buf, int buf_len,
    int timeout_ms)
{
    int rc;

    {
        rc = client->net->read(client->net->context, buf, buf_len, timeout_ms);
    }

#ifdef WOLFMQTT_DEBUG_SOCKET
    if (rc != 0 && rc != MQTT_CODE_CONTINUE) { /* hide in non-blocking case */
        PRINTF("MqttSocket_Read: Len=%d, Rc=%d", buf_len, rc);
    }
#endif

    return rc;
}

int MqttSocket_Read(MqttClient *client, byte* buf, int buf_len, int timeout_ms)
{
    int rc;

    /* Validate arguments */
    if (client == NULL || client->net == NULL || client->net->read == NULL ||
        buf == NULL || buf_len <= 0) {
        return MQTT_CODE_ERROR_BAD_ARG;
    }

    /* check for buffer position overflow */
    if (client->read.pos >= buf_len) {
        return MQTT_CODE_ERROR_OUT_OF_BUFFER;
    }

    do {
        rc = MqttSocket_ReadDo(client, &buf[client->read.pos],
            buf_len - client->read.pos, timeout_ms);
        if (rc <= 0) {
            break;
        }
        client->read.pos += rc;
    } while (client->read.pos < buf_len);

    /* handle return code */
    if (rc > 0) {
        /* return length read and reset position */
        rc = client->read.pos;
        client->read.pos = 0;
    }

    return rc;
}


int MqttSocket_Connect(MqttClient *client, const char* host, word16 port,
    int timeout_ms, int use_tls, MqttTlsCb cb)
{
    int rc = MQTT_CODE_SUCCESS;

    /* Validate arguments */
    if (client == NULL || client->net == NULL ||
        client->net->connect == NULL) {
        return MQTT_CODE_ERROR_BAD_ARG;
    }

#ifndef ENABLE_MQTT_TLS
    /* cannot use TLS unless ENABLE_MQTT_TLS is defined */
    if (use_tls) {
        return MQTT_CODE_ERROR_BAD_ARG;
    }
#endif

    if ((client->flags & MQTT_CLIENT_FLAG_IS_CONNECTED) == 0) {
        /* Validate port */
        if (port == 0) {
            port = (use_tls) ? MQTT_SECURE_PORT : MQTT_DEFAULT_PORT;
        }

        /* Connect to host */
        rc = client->net->connect(client->net->context, host, port, timeout_ms);
        if (rc < 0) {
            return rc;
        }
        client->flags |= MQTT_CLIENT_FLAG_IS_CONNECTED;
    }

    (void)cb;

#ifdef WOLFMQTT_DEBUG_SOCKET
    PRINTF("MqttSocket_Connect: Rc=%d", rc);
#endif

    /* Check for error */
    if (rc < 0) {
        rc = MQTT_CODE_ERROR_NETWORK;
    }

    return rc;
}

int MqttSocket_Disconnect(MqttClient *client)
{
    int rc = MQTT_CODE_SUCCESS;
    if (client) {

        /* Make sure socket is closed */
        if (client->net && client->net->disconnect) {
            rc = client->net->disconnect(client->net->context);
        }
        client->flags &= ~MQTT_CLIENT_FLAG_IS_CONNECTED;
    }
#ifdef WOLFMQTT_DEBUG_SOCKET
    PRINTF("MqttSocket_Disconnect: Rc=%d\n", rc);
#endif

    /* Check for error */
    if (rc < 0) {
        rc = MQTT_CODE_ERROR_NETWORK;
    }

    return rc;
}
