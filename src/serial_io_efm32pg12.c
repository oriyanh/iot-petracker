#include <stdio.h>
#include <string.h>
#include "em_gpio.h"
#include "em_usart.h"
#include "em_leuart.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_timer.h"

#include "config.h"
#include "serial_io.h"
#include "timer_manager.h"
#include "logger.h"

#define CIRCULAR_BUFFER_SIZE          4096

/* Declare a circular buffer structure to use for Rx and Tx queues */
volatile struct circularBuffer
{
  uint8_t  data[CIRCULAR_BUFFER_SIZE];  /* data buffer */
  uint32_t rdI;               /* read index */
  uint32_t wrI;               /* write index */
  uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
  bool     overflow;          /* buffer overflow indicator */
} usartRxBuf, leuartRxBuf, usartTxBuf = { {0}, 0, 0, 0, false };

/**
 * Descriptor for serial device
 */
typedef struct _SerialDesc
{
  char *name;
  volatile struct circularBuffer *rxBuf, *txBuf;
  void* device;
  void *cfg;
  int rxIF,txIF;
  long IFMask;
  bool enabled;
  IRQn_Type rxIRQ, txIRQ;
} SerialDesc;

static SerialDesc leuartDesc = {0}, usartDesc = {0};
static SerialDesc *devices[2] = {&usartDesc, &leuartDesc};

/* Setup USART2 in async mode for RS232 with Cellular Modem */
static USART_TypeDef           * uart   = USART2;
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

/* Setup LEUART0 for RS232 with GPS */
static LEUART_TypeDef *leuart = LEUART0;
static LEUART_Init_TypeDef leuartInit = LEUART_INIT_DEFAULT;


/**
 * Initializes serial device to use USART2 for duplex rx/tx async RS232 mode at PA6 and PA7
 */
static void initUsart2(SerialDesc *ser, unsigned int baudrate)
{
  if (ser == NULL)
  {
    logWarn("NULL SerialDesc");
    return;
  }

  if (ser->enabled)
  {
    logWarn("USART2 already enabled");
    return;
  }

  ser->rxBuf = &usartRxBuf;
  ser->txBuf = &usartTxBuf;
  ser->device = uart;
  ser->cfg = &uartInit;
  ser->rxIF = USART_IF_RXDATAV;
  ser->txIF = USART_IF_TXBL;
  ser->IFMask = _USART_IF_MASK;
  ser->rxIRQ = USART2_RX_IRQn;
  ser->txIRQ = USART2_TX_IRQn;

  CMU_ClockEnable(cmuClock_USART2, true);
  /* Configure GPIO pins */
  GPIO_PinModeSet(gpioPortA, 6, gpioModePushPull, 1);
  GPIO_PinModeSet(gpioPortA, 7, gpioModeInput, 0);

  /* Prepare struct for initializing UART in asynchronous mode*/
  uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
  uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
  uartInit.baudrate     = baudrate;         /* Baud rate */
  uartInit.oversampling = usartOVS16;     /* Oversampling. Range is 4x, 6x, 8x or 16x */
  uartInit.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
  uartInit.parity       = usartNoParity;  /* Parity mode */
  uartInit.stopbits     = usartStopbits1; /* Number of stop bits. Range is 0 to 2 */
  uartInit.mvdis        = false;          /* Disable majority voting */
  uartInit.prsRxEnable  = false;          /* Enable USART Rx via Peripheral Reflex System */
  uartInit.prsRxCh      = usartPrsRxCh0;  /* Select PRS channel if enabled */

  /* Initialize USART with uartInit struct */
  USART_InitAsync(uart, ser->cfg);

  /* Prepare UART Rx and Tx interrupts */
  USART_IntClear(uart, ser->IFMask);
  USART_IntEnable(uart, ser->rxIF);
  NVIC_ClearPendingIRQ(ser->rxIRQ);
  NVIC_ClearPendingIRQ(ser->txIRQ);
  NVIC_EnableIRQ(ser->rxIRQ);
  NVIC_EnableIRQ(ser->txIRQ);
  /* Enable I/O pins at USART2 location #1 */
  uart->ROUTEPEN = USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN;
  uart->ROUTELOC0 = USART_ROUTELOC0_TXLOC_LOC1 | USART_ROUTELOC0_RXLOC_LOC1;

  /* Enable UART */
  USART_Enable(uart, usartEnable);
}

/**
 * Initializes serial device to use LEUART for simplex rx RS232 mode at PD11
 */
static void initLeuart0(SerialDesc *ser, unsigned int baudrate)
{
  if (ser == NULL)
  {
    logWarn("NULL SerialDesc");
    return;
  }

  if (ser->enabled)
  {
    logWarn("LEUART0 already enabled");
    return;
  }

  ser->rxBuf = &leuartRxBuf;
  ser->txBuf = NULL;
  ser->device = leuart;
  ser->cfg = &leuartInit;
  ser->rxIF = LEUART_IF_RXDATAV;
  ser->txIF = 0;
  ser->IFMask = _LEUART_IF_MASK;
  ser->rxIRQ = LEUART0_IRQn;
  ser->txIRQ = 0;
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_LEUART0, true);
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockDivSet(cmuClock_LEUART0, cmuClkDiv_1);
  /* Enable GPIO for LEUART0. RX is on D11 */
  GPIO_PinModeSet(gpioPortD,            /* Port */
                  11,                    /* Port number */
                  gpioModeInputPull,    /* Pin mode is set to input only, with pull direction given bellow */
                  0);                   /* Pull direction is set to pull-up */
  /* Enable CORE LE clock in order to access LE modules */
  leuartInit.enable   = leuartEnableRx;     /* Activate data reception on LEUn_RX pin. */
  leuartInit.refFreq  = 0;                  /* Inherit the clock frequency from the LEUART clock source */
  leuartInit.baudrate = baudrate;               /* Baudrate */
  leuartInit.databits = leuartDatabits8;    /* Each LEUART frame containes 8 databits */
  leuartInit.parity   = leuartNoParity;     /* No parity bits in use */
  leuartInit.stopbits = leuartStopbits1;    /* Setting the number of stop bits in a frame to 2 bitperiods */
  /* Reseting and initializing LEUART1 */
  LEUART_Reset(leuart);
  LEUART_Init(leuart, ser->cfg);

  /* Route LEUART0 RX pin to LOC18 (PD11) */
  LEUART_IntClear(leuart, ser->IFMask);
  LEUART_IntEnable(leuart, ser->rxIF);
  NVIC_ClearPendingIRQ(ser->rxIRQ);
  NVIC_EnableIRQ(ser->rxIRQ);
  leuart->ROUTEPEN = LEUART_ROUTEPEN_RXPEN;
  leuart->ROUTELOC0 = LEUART_ROUTELOC0_RXLOC_LOC18;

  LEUART_Enable(leuart, leuartEnableRx);
}

int SerialInit(SERIAL_PORT port, unsigned int baud, int *handle)
{
  if (handle == NULL)
  {
    logError("SerialInit fail, handle is NULL");
    return -1;
  }

  SerialDesc* ser = NULL;
  switch (port)
  {
    case MODEM_PORT:
      ser = devices[port];
      ser->name = MODEM_PORT_NAME;
      initUsart2(ser, baud);
      break;

    case GPS_PORT:
      ser = devices[port];
      ser->name = GPS_PORT_NAME;
      initLeuart0(ser, baud);
      break;

    default:
      logError("SerialInit Fail, serial port '%d' doesn't exist", port);
      return -1;
  }

  ser->enabled = true;
  *handle = port;
  logInfo("SerialInit success: port='%s'", ser->name);

  return 0;
}

int SerialRecv(unsigned char *buf, unsigned int max_len, unsigned int timeout_ms, int *handle)
{
  // TODO handle buffer overflow better, or at least let the user know that the data is botched
  if (buf == NULL)
  {
    logError("SerialRecv fail, NULL buffer");
    return -1;
  }

  if (handle == NULL)
  {
    logError("SerialRecv fail, handle is NULL");
    return -1;
  }

  if (*handle != MODEM_PORT && *handle != GPS_PORT)
  {
    logError("SerialRecv fail, invalid handle");
    return -1;
  }
  SerialDesc *ser = devices[*handle];

  if (!ser->enabled)
  {
    logError("Serial device not enabled: %s", ser->name);
    return -1;
  }

  int pendingBytes = 0;
  uint32_t totalBytesRecv = 0;
  int timer_d = get_timer();
  enableTimer(timeout_ms, timer_d);
  while ((ser->rxBuf->pendingBytes < max_len) && !isTimedout(timer_d))
  {
    if (ser->rxBuf->overflow)
    {
      ser->rxBuf->overflow = false;
      logInfo("rx buffer overflow");
      break;
    }
  }

  disableTimer(timer_d);
  pendingBytes = ser->rxBuf->pendingBytes;
  if (!pendingBytes)
  {
    return 0;
  }
  /* Copy data from Rx buffer to dataPtr */
  for (; totalBytesRecv < max_len && totalBytesRecv < pendingBytes; totalBytesRecv++)
  {
    *(buf + totalBytesRecv) = ser->rxBuf->data[ser->rxBuf->rdI];
    ser->rxBuf->rdI      = (ser->rxBuf->rdI + 1) % CIRCULAR_BUFFER_SIZE;

  }

  ser->rxBuf->pendingBytes -= totalBytesRecv;

  return totalBytesRecv;
}

int SerialSend(const unsigned char *buf, unsigned int size, int *handle)
{
  if (buf == NULL)
  {
    logError("SerialSend fail, NULL buffer");
    return -1;
  }

  if (handle == NULL)
  {
    logError("SerialSend fail, handle is NULL");
    return -1;
  }

  if (*handle != MODEM_PORT && *handle != GPS_PORT)
  {
    logError("SerialSend fail, invalid handle");
    return -1;
  }
  SerialDesc *ser = devices[*handle];
  if (*handle != MODEM_PORT && *handle != GPS_PORT) {
    return -1;
  }

  if (!ser->enabled)
  {
    logError("Serial device not enabled: %s", ser->name);
    return -1;
  }

  if (size > CIRCULAR_BUFFER_SIZE)
  {
    /* Buffer can never fit the requested amount of data */
    return -1;
  }

  /* Check if buffer has room for new data */
  if ((ser->txBuf->pendingBytes + size) > CIRCULAR_BUFFER_SIZE)
  {
    /* Wait until there is room in tx buffer*/
    while ((ser->txBuf->pendingBytes + size) > CIRCULAR_BUFFER_SIZE);
  }

  /* Fill dataPtr[0:dataLen-1] into txBuffer */
  uint32_t i = 0;
  while (i < size)
  {
    ser->txBuf->data[ser->txBuf->wrI] = *(buf + i);
    ser->txBuf->wrI             = (ser->txBuf->wrI + 1) % CIRCULAR_BUFFER_SIZE;
    i++;
  }

  /* Increment pending byte counter */
  ser->txBuf->pendingBytes += size;

  /* Enable interrupt on USART TX Buffer*/
  USART_IntEnable((USART_TypeDef*)ser->device, ser->txIF);
  return i;
}

void SerialFlushInputBuff(int *handle) {
  if (handle == NULL)
  {
    logError("SerialFlushInputBuff fail, handle is NULL");
    return;
  }

  if (*handle != MODEM_PORT && *handle != GPS_PORT)
  {
    logError("SerialFlushInputBuff fail, invalid handle");
    return;
  }
  SerialDesc *ser = devices[*handle];

  if (!ser->enabled)
  {
    logError("SerialFlushInputBuff fail, Serial device not enabled: %s", ser->name);
    return;
  }

  ser->rxBuf->pendingBytes = 0;
  ser->rxBuf->wrI = ser->rxBuf->rdI;
  ser->rxBuf->overflow = false;
}

int SerialDisable(int *handle) {
  logInfo("SerialDisable");
  if (handle == NULL)
  {
    logError("SerialDisable fail, handle is NULL");
    return -1;
  }

  if (*handle != MODEM_PORT && *handle != GPS_PORT)
  {
    logError("SerialDisable fail, invalid handle");
    return -1;
  }

  SerialDesc *ser = devices[*handle];
  if (!ser->enabled)
  {
    logWarn("Serial port %s already closed", ser->name);
    return 0;
  }

  SerialFlushInputBuff(handle);
  NVIC_ClearPendingIRQ(ser->rxIRQ);
  switch (*handle)
  {
    case MODEM_PORT:
      USART_IntClear((USART_TypeDef*)ser->device, ser->IFMask);
      USART_IntDisable((USART_TypeDef*)ser->device, ser->rxIF);
      NVIC_ClearPendingIRQ(ser->txIRQ);
      USART_Enable((USART_TypeDef*)ser->device, usartDisable);
      break;
    case GPS_PORT:
      LEUART_IntClear((LEUART_TypeDef*)ser->device, ser->IFMask);
      LEUART_IntDisable((LEUART_TypeDef*)ser->device, ser->rxIF);
      LEUART_Enable((LEUART_TypeDef*)ser->device, leuartDisable);
      break;
    default:
      break;
  }

  logInfo("SerialDisable success: %s", ser->name);
  ser->name = NULL;
  ser->device = NULL;
  ser->cfg = NULL;
  ser->rxBuf = NULL;
  ser->txBuf = NULL;
  ser->enabled = false;
  return 0;
}

/**
 * Copies rx byte from device register to pending rx data buffer
 */
static void UART_RX_Handler(SerialDesc* ser, uint8_t rxData)
{
  /* Copy data into RX Buffer */
  ser->rxBuf->data[ser->rxBuf->wrI] = rxData;
  ser->rxBuf->wrI             = (ser->rxBuf->wrI + 1) % CIRCULAR_BUFFER_SIZE;
  ser->rxBuf->pendingBytes++;

  /* Flag Rx overflow */
  if (ser->rxBuf->pendingBytes > CIRCULAR_BUFFER_SIZE)
  {
    ser->rxBuf->overflow = true;
  }
}

/**************************************************************************//**
 * @brief USART2 RX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 * Note that this function handles overflows in a very simple way.
 *
 *****************************************************************************/
void USART2_RX_IRQHandler(void)
{
  /* Check for RX data valid interrupt */
  SerialDesc* ser = &usartDesc;
  USART_TypeDef* dev = (USART_TypeDef*)ser->device;
  if (dev->STATUS & USART_STATUS_RXDATAV)
  {
    /* Copy data into RX Buffer */
    uint8_t rxData = USART_Rx(dev);
    UART_RX_Handler(ser, rxData);
    /* Clear RX interrupt */
    USART_IntClear(dev, ser->rxIF);
  }
}

/**************************************************************************//**
 * @brief LEUART0 RX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 * Note that this function handles overflows in a very simple way.
 *
 *****************************************************************************/
void LEUART0_IRQHandler(void)
{
  /* Check for RX data valid interrupt */
  SerialDesc* ser = &leuartDesc;
  LEUART_TypeDef* dev = (LEUART_TypeDef*)ser->device;
  if (dev->STATUS & LEUART_STATUS_RXDATAV)
  {
    /* Copy data into RX Buffer */
    uint8_t rxData = LEUART_Rx(dev);
    UART_RX_Handler(ser, rxData);
    /* Clear RX interrupt */
    LEUART_IntClear(dev, ser->rxIF);
  }
}

/**************************************************************************//**
 * @brief USART2 TX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 *****************************************************************************/
void USART2_TX_IRQHandler(void)
{
  SerialDesc* ser = &usartDesc;
  USART_TypeDef* dev = (USART_TypeDef*)ser->device;
  /* Clear interrupt flags by reading them. */
  USART_IntGet(dev);

  /* Check TX buffer level status */
  if (dev->STATUS & USART_STATUS_TXBL)
  {
    if (ser->txBuf->pendingBytes > 0)
    {
      /* Transmit pending character */
      USART_Tx(dev, ser->txBuf->data[usartTxBuf.rdI]);
      ser->txBuf->rdI = (ser->txBuf->rdI + 1) % CIRCULAR_BUFFER_SIZE;
      ser->txBuf->pendingBytes--;
    }

    /* Disable Tx interrupt if no more bytes in queue */
    if (ser->txBuf->pendingBytes == 0)
    {
      USART_IntDisable(dev, ser->txIF);
    }
  }
}

