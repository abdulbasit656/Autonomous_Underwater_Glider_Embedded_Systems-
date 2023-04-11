
// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef TIMER_H
#define	TIMER_H

#include <xc.h> // include processor files - each processor file is guarded.  

#define TIMER1 1
#define TIMER2 2
#define TIMER3 3

int tmr_setup_period(int timer, int ms);
int tmr_wait_period(int timer);
int tmr_wait_ms(int timer, int ms);
void tmr2_restart_timer();

#endif	/* TIMER_H */

