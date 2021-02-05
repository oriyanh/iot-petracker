#include "timer_manager.h"
#include "em_timer.h"
#include "em_cmu.h"



TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
#define NUM_OF_TIMERS 2
#define TOP 13671
static const int MILISEC_FRAGMENT = TOP / 1000;
static volatile uint32_t msTicks = 0; /* counts 1ms timeTicks */

typedef struct{
	int top;
	TIMER_TypeDef* timer;
	volatile bool timedOut;
	volatile bool enable;
}TimerStruct;


static volatile TimerStruct timers[2] = {0};

/**************************************************************************//**
 * @brief Setting up timer + interrupts for handling timeouts in SerialRecv(), prior to use
 *
 */
void setupTimer()
{
	/* Enable clock for TIMER0 module = HFPER and TIMER0 */
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);

	/* Setup SysTick Timer for 1 msec interrupts  */
	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) {
		while (1) ;
	}

	/* Select TIMER0 parameters */
	timerInit.enable = true;
	timerInit.debugRun = true;
	timerInit.prescale   = timerPrescale1024;
	timerInit.clkSel     = timerClkSelHFPerClk;
	timerInit.fallAction     = timerInputActionNone;
	timerInit.riseAction     = timerInputActionNone;
	timerInit.mode     = timerModeUp;
	timerInit.dmaClrAct     = false;
	timerInit.quadModeX4     = false;
	timerInit.oneShot     = true;
	timerInit.sync     = false;

	timers[0].timer = TIMER0;
	timers[1].timer = TIMER1;

	for(int i=0; i<NUM_OF_TIMERS;i++){
		timers[i].top = TOP;
		timers[i].timedOut = false;
		timers[i].enable = false;
		TIMER_IntEnable(timers[i].timer, TIMER_IF_OF);
		TIMER_TopSet(timers[i].timer, TOP);

	}

	/* Enable TIMER0 interrupt vector in NVIC */
	NVIC_EnableIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);

	/* Set TIMER Top value */


}

int get_timer(void){
	for(int i=0; i<NUM_OF_TIMERS; i++){
		if (!timers[i].enable){
			return i;
		}
	}
	return -1;
}

/**************************************************************************//**
 * @brief Setting up timer + interrupts for handling timeouts in SerialRecv(), prior to use
 *
 * @param timeout_ms timeout in ms
 * @param enable whether to start the timer or stop it
 */
void enableTimer(uint32_t timeout_ms, int timerDescriptor)
{
	timers[timerDescriptor].enable = true;
	TIMER_TopSet(timers[timerDescriptor].timer, MILISEC_FRAGMENT*timeout_ms);
	timers[timerDescriptor].timedOut = false;
	/* Configure and start TIMER */
	TIMER_Init(timers[timerDescriptor].timer, &timerInit);
	TIMER_Enable(timers[timerDescriptor].timer, true);

}

void disableTimer(int timerDescriptor){
	timers[timerDescriptor].enable = false;
	TIMER_Enable(timers[timerDescriptor].timer, false);
}

bool isTimedout(int timerDescriptor){
	return timers[timerDescriptor].timedOut;
}

void sleepMs(uint32_t milisecs){
  uint32_t wakeupT = msTicks + milisecs;
  if (wakeupT < msTicks)
  {
    while (msTicks > wakeupT);
  }
  while (msTicks <= wakeupT);
}

void TIMER0_IRQHandler(void)
{
	/* Clear flag for TIMER0 overflow interrupt */
	TIMER_IntClear(timers[0].timer, TIMER_IF_OF);
	timers[0].timedOut = true;
}

void TIMER1_IRQHandler(void)
{
	/* Clear flag for TIMER1 overflow interrupt */
	TIMER_IntClear(timers[1].timer, TIMER_IF_OF);
	timers[1].timedOut = true;
}


/***************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 ******************************************************************************/
void SysTick_Handler(void)
{
	++msTicks;
}
