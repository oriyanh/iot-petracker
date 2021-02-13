# iot-ex9

## Authors
Oriyan Hermoni oriyan.hermoni@mail.huji.ac.il , 302170204<br> 
Maya Lulko maya.lulko@mail.huji.ac.il , 312414089

## Package contents:
* README.md
* iot_ex9.sls
* iot_ex9.bin


### Run
After compiling the project, simply execute `main.c`.

## Design
Our main does:
 1. Initialization of:
  a. clocks (HFXO and LFA)
  b. The display screen.
  c. PB0 and PB1 interrupts
 2. Running of infinity loop that calls different functions that will be explained in the next section.
    a. Check if applicationMode is switched to SEND_UART_MODE (which happens when clicking on pb0) - if yes, in initiates the following:
	- Disables the pb0 and pb1 interrupts.
	- Initiates MQTT client. This step mainly includes validating connection to modem and registering to network. See previous exercises for more details.
	- Opens a socket and connects to brocker
	- Publishes the message with the network's information, ccid, csq and number of pb1 clicks since last pressed pb0.
	- enables the pb0 and pb1 interrupts.
    b. If `redrawScreen` is true - call drawScreen()
        1. unset redrawScreen
        2. Clear display
        3. redraw the screen according to the the state of the applicationMode.

<br>
Different "flags" or global variables changed within interrupt-handlers:
 a. applicationMode - enum of type APP_MODE with possible values MENU_DISPLAY_MODE, SEND_UART_MODE, CLICKS_DISPLAY_MODE, this value changes via system interrupts (GPIO clicks or timeouts)
 b. redrawScreen - a boolean to state whether we need to update the display. The display is updated according to applicationMode.
 c. pb1Clicks - Number of PB1 clicks since last PB0 click

<br>
Interrupts and handlers:
* GPIO_Unified_IRQ - gets the interrupt caused by pressing on one of the buttons, BSP_GPIO_PB0_PIN and BSP_GPIO_PB1_PIN.
* USART2_RX_IRQHandler - handles USART_IF_RXDATAV which is set whenever a byte arrives over USART2
* USART2_TX_IRQHandler - sends a single byte whenver USART_IF_TXBL is set, which is set in SW by SerialSend()
* TIMER0_IRQHandler/TIMER1_IRQHandler - handle timeouts for SerialRecv (for example)
* SysTick_Handler - used for sleepMs()
<br>
Timers:
We created a separate library inorder to manage the different timers we need in this project called `timer_manager.h`.
Examples for uses of the timer: Do interrupt whenever we have reached the command timeout, or when doing "sleep" command through out the code to ensure we gave enough time between the commands.
The way it works, each time a library such as serial or cellular want to use a timer, they request for one and receive a number 0 or 1.
This number, indicates the id of the timer assigned to them (TIMER0 or TIMER1 respectively). The libraries also can enable the timers, ask if they are timedout, and disable them.
This helps through out the project to manage our timers use. 
<br>
Collection of Metadata:
In order to collect the data for the message we wish to send, we have created a struct called `MODEM_METADATA` that holds: ccid, csq and a OPARATOR_INFO struct.
We collect the data after successfully registering to a network. We added additional checks to validate our connection.

 
