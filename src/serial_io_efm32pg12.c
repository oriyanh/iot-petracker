#include <stdio.h>
#include <string.h>
#include "em_gpio.h"
#include "em_usart.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_timer.h"

#include "timer_manager.h"
#include "serial_io.h"
#include "logger.h"

/* Setup UART2 in async mode for RS232*/
static USART_TypeDef           * uart   = USART2;
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

#define BUFFERSIZE          1024

/* Declare a circular buffer structure to use for Rx and Tx queues */
volatile struct circularBuffer
{
	uint8_t  data[BUFFERSIZE];  /* data buffer */
	uint32_t rdI;               /* read index */
	uint32_t wrI;               /* write index */
	uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
	bool     overflow;          /* buffer overflow indicator */
} rxBuf, txBuf = { {0}, 0, 0, 0, false };


/**
 * @brief Initialises the serial connection.
 * @param port - unused
 * @param baud - the baud rate of the communication. For example: 9600, 115200
 * @return 0 if succeeded in opening the port and -1 otherwise.
 */
int SerialInit(const char* port, unsigned int baud)
{
	CMU_ClockEnable(cmuClock_USART2, true);
	/* Configure GPIO pins */
	GPIO_PinModeSet(gpioPortA, 6, gpioModePushPull, 1);
	GPIO_PinModeSet(gpioPortA, 7, gpioModeInput, 0);

	/* Prepare struct for initializing UART in asynchronous mode*/
	uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
	uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
	uartInit.baudrate     = baud;         /* Baud rate */
	uartInit.oversampling = usartOVS16;     /* Oversampling. Range is 4x, 6x, 8x or 16x */
	uartInit.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
	uartInit.parity       = usartNoParity;  /* Parity mode */
	uartInit.stopbits     = usartStopbits1; /* Number of stop bits. Range is 0 to 2 */
	uartInit.mvdis        = false;          /* Disable majority voting */
	uartInit.prsRxEnable  = false;          /* Enable USART Rx via Peripheral Reflex System */
	uartInit.prsRxCh      = usartPrsRxCh0;  /* Select PRS channel if enabled */

	/* Initialize USART with uartInit struct */
	USART_InitAsync(uart, &uartInit);

	/* Prepare UART Rx and Tx interrupts */
	USART_IntClear(uart, _USART_IF_MASK);
	USART_IntEnable(uart, USART_IF_RXDATAV);
	NVIC_ClearPendingIRQ(USART2_RX_IRQn);
	NVIC_ClearPendingIRQ(USART2_TX_IRQn);
	NVIC_EnableIRQ(USART2_RX_IRQn);
	NVIC_EnableIRQ(USART2_TX_IRQn);
	/* Enable I/O pins at USART2 location #1 */
	uart->ROUTEPEN = USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN;
	uart->ROUTELOC0 = USART_ROUTELOC0_TXLOC_LOC1 | USART_ROUTELOC0_RXLOC_LOC1;

	/* Enable UART */
	USART_Enable(uart, usartEnable);
	return 0;
}

/**
 * @brief Receives data from serial connection.
 * @param buf - the buffer that receives the input.
 * @param max_len - maximum bytes to read into buf (buf must be equal or greater than max_len).
 * @param timeout_ms - read operation timeout milliseconds.
 * @return amount of bytes read into buf, -1 on error.
 */
int SerialRecv(unsigned char *buf, unsigned int max_len, unsigned int timeout_ms) {
	if (buf == NULL) {
		return -1;
	}

	int pendingBytes = 0;
	uint32_t totalBytesRecv = 0;
	int timer_d = get_timer();
	enableTimer(timeout_ms, timer_d);
	while ((rxBuf.pendingBytes < max_len) && !isTimedout(timer_d)) {
		if (rxBuf.overflow){
			rxBuf.overflow = false;
			printf("rx buffer overflow.\n");
			break;
		}
	}
	disableTimer(timer_d);
	pendingBytes = rxBuf.pendingBytes;
	if (!pendingBytes) {
		return 0;
	}
	/* Copy data from Rx buffer to dataPtr */
	for (; totalBytesRecv < max_len && totalBytesRecv < pendingBytes; totalBytesRecv++)
	{

		*(buf + totalBytesRecv) = rxBuf.data[rxBuf.rdI];
		rxBuf.rdI      = (rxBuf.rdI + 1) % BUFFERSIZE;

	}

	rxBuf.pendingBytes -= totalBytesRecv;

	return totalBytesRecv;
}

/**
 * @brief Sends data through the serial connection.
 * @param buf - the buffer that contains the data to send
 * @param size - number of bytes to send
 * @return amount of bytes written into buf, -1 on error
 */
int SerialSend(const unsigned char *buf, unsigned int size) {
	if (buf == NULL) {
		return -1;
	}

	uint32_t i = 0;
	if (size > BUFFERSIZE)
	{
		/* Buffer can never fit the requested amount of data */
		return -1;
	}

	/* Check if buffer has room for new data */
	if ((txBuf.pendingBytes + size) > BUFFERSIZE)
	{
		/* Wait until room */
		while ((txBuf.pendingBytes + size) > BUFFERSIZE) ;
	}

	/* Fill dataPtr[0:dataLen-1] into txBuffer */
	while (i < size)
	{
		txBuf.data[txBuf.wrI] = *(buf + i);
		txBuf.wrI             = (txBuf.wrI + 1) % BUFFERSIZE;
		i++;
	}

	/* Increment pending byte counter */
	txBuf.pendingBytes += size;

	/* Enable interrupt on USART TX Buffer*/
	USART_IntEnable(uart, USART_IF_TXBL);
	return i;
}


/**
 * @brief Empties the input buffer.
 */
void SerialFlushInputBuff(void) {
	rxBuf.pendingBytes = 0;
	rxBuf.wrI = rxBuf.rdI;
}

/**
 * @brief Disable the serial connection.
 * @return 0 if succeeded in closing the port and -1 otherwise.
 */
int SerialDisable(void) {
	logInfo("Closing serial connection");

	/* Enable overflow interrupt */
	TIMER_IntClear(TIMER0, TIMER_IF_OF);
	TIMER_IntDisable(TIMER0, TIMER_IF_OF);
	NVIC_ClearPendingIRQ(TIMER0_IRQn);

	SerialFlushInputBuff();
	USART_IntClear(uart, _USART_IF_MASK);
	USART_IntDisable(uart, USART_IF_RXDATAV);
	NVIC_ClearPendingIRQ(USART2_RX_IRQn);
	NVIC_ClearPendingIRQ(USART2_TX_IRQn);

	USART_Enable(uart, usartDisable);

	return 0;
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
	/* Check for RX data valid interrupt */
	if (uart->STATUS & USART_STATUS_RXDATAV)
	{
		/* Copy data into RX Buffer */
		uint8_t rxData = USART_Rx(uart);
		rxBuf.data[rxBuf.wrI] = rxData;
		rxBuf.wrI             = (rxBuf.wrI + 1) % BUFFERSIZE;
		rxBuf.pendingBytes++;

		/* Flag Rx overflow */
		if (rxBuf.pendingBytes > BUFFERSIZE)
		{
			rxBuf.overflow = true;
		}

		/* Clear RXDATAV interrupt */
		USART_IntClear(USART2, USART_IF_RXDATAV);
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
	/* Clear interrupt flags by reading them. */
	USART_IntGet(uart);

	/* Check TX buffer level status */
	if (uart->STATUS & USART_STATUS_TXBL)
	{
		if (txBuf.pendingBytes > 0)
		{
			/* Transmit pending character */
			USART_Tx(uart, txBuf.data[txBuf.rdI]);
			txBuf.rdI = (txBuf.rdI + 1) % BUFFERSIZE;
			txBuf.pendingBytes--;
		}

		/* Disable Tx interrupt if no more bytes in queue */
		if (txBuf.pendingBytes == 0)
		{
			USART_IntDisable(uart, USART_IF_TXBL);
		}
	}
}

