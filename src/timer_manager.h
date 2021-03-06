#ifndef IOT_TIMER_MANAGER_H
#define IOT_TIMER_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

/*
 * @brief Setting up timers + interrupts for handling timeouts, prior to use
 * does timer setup that populates timerinit struct and sets the clocks.
 */
void setupTimer();

/*
 * returns 0 or 1 to represent the id of an available timer that has been opened.
 */
int get_timer(void);

/*
 * returns if timeout has occurred
 */
bool isTimedout(int timerDescriptor);

/**
 * @brief Enables timeout for this timer
 *
 * @param timeout_ms timeout in ms
 * @param enable whether to start the timer or stop it
 */
void enableTimer(uint32_t timeout_ms, int timerDescriptor);

/*
 * disables timer
 */
void disableTimer(int timerDescriptor);

/*
 * sleep for X milisecs
 */
void sleepMs(uint32_t milisecs);

#endif //IOT_TIMER_MANAGER_H
