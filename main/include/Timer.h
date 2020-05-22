/*
 * Timer.h
 *
 *  Created on: May 21, 2020
 *      Author: jose
 */

#ifndef MAIN_INCLUDE_TIMER_H_
#define MAIN_INCLUDE_TIMER_H_

#include "driver/timer.h"
/////////////////*******Define***********//////////////////////////
#define BAUD_RATE                    							115200
#define SILENCE_BYTES                                           150
#define TIMER_BITS_US											(double)((SILENCE_BYTES*10*1000000)/BAUD_RATE)
#define TIMER_SCALE_US           								(TIMER_BASE_CLK / (TIMER_DIVIDER*1000000))  // convert counter value to seconds

#define TIMER_INTERVAL0_SEC   							    	5   // sample test interval for the second timer
#define WITH_RELOAD      										1

#define TIMER_DIVIDER         									16  //  Hardware timer clock divider
#define TIMER_SCALE           									(TIMER_BASE_CLK / TIMER_DIVIDER)  // seconds

void timer_config(int timer_idx,bool auto_reload, double timer_interval_sec);




#endif /* MAIN_INCLUDE_TIMER_H_ */



