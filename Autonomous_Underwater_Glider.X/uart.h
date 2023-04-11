
// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef UART_H
#define	UART_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include "circularBuffer.h"

#define FCY         1843200     // Number of clocks per seconds

void UART_setup(int baudrate, volatile circular_buffer_t *inBuffer, volatile circular_buffer_t *outBuffer);
void UARTget();
void UARTsend(char* msg);

#endif	/* UART_H */

