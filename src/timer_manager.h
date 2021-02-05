#ifndef IOT_EX9_TIMERMANAGER_H
#define IOT_EX9_TIMERMANAGER_H

#include <stdint.h>
#include <stdbool.h>

/*
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

/*
 * enables timer with the given timout
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

#endif //IOT_EX9_TIMERMANAGER_H
