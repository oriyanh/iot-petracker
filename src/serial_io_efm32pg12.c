#include <stdio.h>
#include <string.h>
#include "em_gpio.h"
#include "em_usart.h"
#include "em_leuart.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "config.h"
#include "timer_manager.h"
#include "serial_io.h"
#include "logger.h"

/* Setup UART2 in async mode for RS232*/
static USART_TypeDef           * uart   = USART2;
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;
static LEUART_TypeDef *leuart = LEUART0;
static LEUART_Init_TypeDef leuartInit = LEUART_INIT_DEFAULT;

#define BUFFERSIZE          4096

/* Declare a circular buffer structure to use for Rx and Tx queues */
volatile struct circularBuffer
{
  uint8_t  data[BUFFERSIZE];  /* data buffer */
  uint32_t rdI;               /* read index */
  uint32_t wrI;               /* write index */
  uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
  bool     overflow;          /* buffer overflow indicator */
} usartRxBuf, leuartRxBuf, usartTxBuf = { {0}, 0, 0, 0, false };

typedef int (*initFunc)(int);
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

static SerialDesc leuartDesc, usartDesc;
static SerialDesc *devices[2] = {&usartDesc, &leuartDesc};

static void initUsart2(SerialDesc *ser, unsigned int baudrate)
{
  if (ser == NULL)
  {
      logWarn("NULL SerialDesc");
      return;
  }
  if (ser->enabled)
  {
      logWarn("Usart2 already enabled");
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


static void initLeuart0(SerialDesc *ser, unsigned int baudrate)
{
  if (ser == NULL)
  {
      logWarn("NULL SerialDesc");
      return;
  }
  if (ser->enabled)
    {
        logWarn("Usart2 already enabled");
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
  /* Enable GPIO for LEUART1. RX is on C7 */
  GPIO_PinModeSet(gpioPortD,            /* Port */
		  11,                    /* Port number */
		  gpioModeInputPull,    /* Pin mode is set to input only, with pull direction given bellow */
		  0);                   /* Pull direction is set to pull-up */
  /* Enable CORE LE clock in order to access LE modules */
  leuartInit.enable   = leuartEnableRx;     /* Activate data reception on LEUn_RX pin. */
  leuartInit.refFreq  = 0;                  /* Inherit the clock frequency from the LEUART clock source */
  leuartInit.baudrate = baudrate;               /* Baudrate = 9600 bps */
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
  leuart->ROUTELOC0 = LEUART_ROUTELOC0_RXLOC_LOC18; // LEUART_IF_RXDATAV
  
  LEUART_Enable(leuart, leuartEnableRx);
}

/**
 * @brief Initialises the serial connection.
 * @param port - unused
 * @param baud - the baud rate of the communication. For example: 9600, 115200
 * @return 0 if succeeded in opening the port and -1 otherwise.
 */
int SerialInit(SERIAL_PORT port, unsigned int baud, int *handle)
{
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
      break;
  }
  ser->enabled = true;
  *handle = port;
  logInfo("SerialInit enabled port '%s'", ser->name);
//  CMU_ClockEnable(cmuClock_USART2, true);
//  /* Configure GPIO pins */
//  GPIO_PinModeSet(gpioPortA, 6, gpioModePushPull, 1);
//  GPIO_PinModeSet(gpioPortA, 7, gpioModeInput, 0);
//
//  /* Prepare struct for initializing UART in asynchronous mode*/
//  uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
//  uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
//  uartInit.baudrate     = baud;         /* Baud rate */
//  uartInit.oversampling = usartOVS16;     /* Oversampling. Range is 4x, 6x, 8x or 16x */
//  uartInit.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
//  uartInit.parity       = usartNoParity;  /* Parity mode */
//  uartInit.stopbits     = usartStopbits1; /* Number of stop bits. Range is 0 to 2 */
//  uartInit.mvdis        = false;          /* Disable majority voting */
//  uartInit.prsRxEnable  = false;          /* Enable USART Rx via Peripheral Reflex System */
//  uartInit.prsRxCh      = usartPrsRxCh0;  /* Select PRS channel if enabled */
//
//  /* Initialize USART with uartInit struct */
//  USART_InitAsync(uart, &uartInit);
//
//  /* Prepare UART Rx and Tx interrupts */
//  USART_IntClear(uart, _USART_IF_MASK);
//  USART_IntEnable(uart, USART_IF_RXDATAV);
//  NVIC_ClearPendingIRQ(USART2_RX_IRQn);
//  NVIC_ClearPendingIRQ(USART2_TX_IRQn);
//  NVIC_EnableIRQ(USART2_RX_IRQn);
//  NVIC_EnableIRQ(USART2_TX_IRQn);
//  /* Enable I/O pins at USART2 location #1 */
//  uart->ROUTEPEN = USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN;
//  uart->ROUTELOC0 = USART_ROUTELOC0_TXLOC_LOC1 | USART_ROUTELOC0_RXLOC_LOC1;
//
//  /* Enable UART */
//  USART_Enable(uart, usartEnable);
  return 0;
}

/**
 * @brief Receives data from serial connection.
 * @param buf - the buffer that receives the input.
 * @param max_len - maximum bytes to read into buf (buf must be equal or greater than max_len).
 * @param timeout_ms - read operation timeout milliseconds.
 * @return amount of bytes read into buf, -1 on error.
 */
int SerialRecv(unsigned char *buf, unsigned int max_len, unsigned int timeout_ms, int *handle) {
  if (buf == NULL || handle == NULL || (*handle != MODEM_PORT && *handle != GPS_PORT)) {
      return -1;
  }

  SerialDesc *ser = devices[*handle];
  int pendingBytes = 0;
  uint32_t totalBytesRecv = 0;
  int timer_d = get_timer();
  enableTimer(timeout_ms, timer_d);
  while ((ser->rxBuf->pendingBytes < max_len) && !isTimedout(timer_d)) {
      if (ser->rxBuf->overflow){
		  ser->rxBuf->overflow = false;
		  logInfo("rx buffer overflow.\n");
		  break;
      }
  }
  disableTimer(timer_d);
  pendingBytes = ser->rxBuf->pendingBytes;
  if (!pendingBytes) {
      return 0;
  }
  /* Copy data from Rx buffer to dataPtr */
  for (; totalBytesRecv < max_len && totalBytesRecv < pendingBytes; totalBytesRecv++)
    {

      *(buf + totalBytesRecv) = ser->rxBuf->data[ser->rxBuf->rdI];
      ser->rxBuf->rdI      = (ser->rxBuf->rdI + 1) % BUFFERSIZE;

    }

  ser->rxBuf->pendingBytes -= totalBytesRecv;

  return totalBytesRecv;
}

/**
 * @brief Sends data through the serial connection.
 * @param buf - the buffer that contains the data to send
 * @param size - number of bytes to send
 * @return amount of bytes written into buf, -1 on error
 */
int SerialSend(const unsigned char *buf, unsigned int size, int *handle) {
  if (buf == NULL || handle == NULL || (*handle != MODEM_PORT && *handle != GPS_PORT)) {
        return -1;
    }
  SerialDesc *ser = devices[*handle];
  if (!ser->enabled)
  {
      logError("Serial device not enabled: %s", ser->name);
      return -1;
  }
  uint32_t i = 0;
  if (size > BUFFERSIZE)
    {
      /* Buffer can never fit the requested amount of data */
      return -1;
    }

  /* Check if buffer has room for new data */
  if ((ser->txBuf->pendingBytes + size) > BUFFERSIZE)
    {
      /* Wait until room */
      while ((ser->txBuf->pendingBytes + size) > BUFFERSIZE) ;
    }

  /* Fill dataPtr[0:dataLen-1] into usartTxBuffer */
  while (i < size)
    {
      ser->txBuf->data[ser->txBuf->wrI] = *(buf + i);
      ser->txBuf->wrI             = (ser->txBuf->wrI + 1) % BUFFERSIZE;
      i++;
    }

  /* Increment pending byte counter */
  ser->txBuf->pendingBytes += size;

  /* Enable interrupt on USART TX Buffer*/
  USART_IntEnable((USART_TypeDef*)ser->device, ser->txIF);
  return i;
}


/**
 * @brief Empties the input buffer.
 */
void SerialFlushInputBuff(int *handle) {
  if (handle == NULL || (*handle != MODEM_PORT && *handle != GPS_PORT)) {
        logWarn("SerialFlushInputBuff invalid handle");
        return;
  }

  SerialDesc *ser = devices[*handle];
  if (!ser->enabled)
  {
    return;
  }
  ser->rxBuf->pendingBytes = 0;
  ser->rxBuf->wrI = ser->rxBuf->rdI;
  ser->rxBuf->overflow = false;
}

/**
 * @brief Disable the serial connection.
 * @return 0 if succeeded in closing the port and -1 otherwise.
 */
int SerialDisable(int *handle) {
  logInfo("SerialDisable");
  
  if (handle == NULL || (*handle != MODEM_PORT && *handle != GPS_PORT)) {
      logWarn("SerialDisable invalid handle");
      return -1;
    }

  SerialDesc *ser = devices[*handle];
  if (!ser->enabled)
  {
      logWarn("Serial port %s already closed", ser->name);
    return 0;
  }
  /* Enable overflow interrupt */

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


void UART_RX_Handler(SerialDesc* ser, uint8_t rxData)
{
  /* Copy data into RX Buffer */
  ser->rxBuf->data[ser->rxBuf->wrI] = rxData;
  ser->rxBuf->wrI             = (ser->rxBuf->wrI + 1) % BUFFERSIZE;
  ser->rxBuf->pendingBytes++;

  /* Flag Rx overflow */
  if (ser->rxBuf->pendingBytes > BUFFERSIZE)
  {
      ser->rxBuf->overflow = true;
  }
}

/**************************************************************************//**
 * @brief UART1 RX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 * Note that this function handles overflows in a very simple way.
 *
 *****************************************************************************/
void USART2_RX_IRQHandler(void)
{
//  /* Check for RX data valid interrupt */
//  if (uart->STATUS & USART_STATUS_RXDATAV)
//    {
//      /* Copy data into RX Buffer */
//      uint8_t rxData = USART_Rx(uart);
//      usartRxBuf.data[usartRxBuf.wrI] = rxData;
//      usartRxBuf.wrI             = (usartRxBuf.wrI + 1) % BUFFERSIZE;
//      usartRxBuf.pendingBytes++;
//
//      /* Flag Rx overflow */
//      if (usartRxBuf.pendingBytes > BUFFERSIZE)
//	{
//	  usartRxBuf.overflow = true;
//	}
//
//      /* Clear RXDATAV interrupt */
//      USART_IntClear(USART2, USART_IF_RXDATAV);
//    }
  
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


void LEUART0_IRQHandler(void)
{
  /* Store and reset pending interupts */
//  leuartif = LEUART_IntGet(LEUART1);
//  LEUART_IntClear(LEUART1, leuartif);
//
//  /* Signal frame found. */
//  if (leuartif & LEUART_IF_SIGF)
//  {
//    /* Zero-terminate rx buffer */
//    len            = BUF_MAX - 1 - ((dmaControlBlock->CTRL >> 4) & 0x3FF);
//    rxbuf[len - 1] = 0;
//
//    /* Reactivate DMA */
//    DMA_ActivateBasic(DMA_CHANNEL,     /* Activate DMA channel 0 */
//                      true,            /* Activate using primary descriptor */
//                      false,           /* No DMA burst */
//                      NULL,            /* Keep source */
//                      NULL,            /* Keep destination */
//                      BUF_MAX - 1);    /* Number of DMA transfer elements (minus 1) */
//
//    /* Write recieved string to LCD */
//    SegmentLCD_Write(rxbuf);
//  }
//  
  /* Check for RX data valid interrupt */
//  if (leuart->STATUS & LEUART_STATUS_RXDATAV)
//  {
//    /* Copy data into RX Buffer */
//    uint8_t rxData = LEUART_Rx(uart);
//    leuartRxBuf.data[leuartRxBuf.wrI] = rxData;
//    leuartRxBuf.wrI             = (leuartRxBuf.wrI + 1) % BUFFERSIZE;
//    leuartRxBuf.pendingBytes++;
//
//    /* Flag Rx overflow */
//    if (leuartRxBuf.pendingBytes > BUFFERSIZE)
//    {
//	leuartRxBuf.overflow = true;
//    }
//
//    /* Clear RXDATAV interrupt */
//    LEUART_IntClear(leuart, LEUART_IF_RXDATAV);
//  }
  
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
 * @brief UART1 TX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 *****************************************************************************/
void USART2_TX_IRQHandler(void)
{
//  /* Clear interrupt flags by reading them. */
//  USART_IntGet(uart);
//
//  /* Check TX buffer level status */
//  if (uart->STATUS & USART_STATUS_TXBL)
//    {
//      if (usartTxBuf.pendingBytes > 0)
//	{
//	  /* Transmit pending character */
//	  USART_Tx(uart, usartTxBuf.data[usartTxBuf.rdI]);
//	  usartTxBuf.rdI = (usartTxBuf.rdI + 1) % BUFFERSIZE;
//	  usartTxBuf.pendingBytes--;
//	}
//
//      /* Disable Tx interrupt if no more bytes in queue */
//      if (usartTxBuf.pendingBytes == 0)
//	{
//	  USART_IntDisable(uart, USART_IF_TXBL);
//	}
//    }

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
      ser->txBuf->rdI = (ser->txBuf->rdI + 1) % BUFFERSIZE;
      ser->txBuf->pendingBytes--;
    }

    /* Disable Tx interrupt if no more bytes in queue */
    if (ser->txBuf->pendingBytes == 0)
    {
      USART_IntDisable(dev, ser->txIF);
    }
  }
}

