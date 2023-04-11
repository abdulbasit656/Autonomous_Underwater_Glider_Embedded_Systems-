
// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef SPI_H
#define	SPI_H

#include <xc.h> // include processor files - each processor file is guarded.  
#define FirstRow 0
#define SecRow 1

void SPI_setup();
void put_char_SPI(char c);
void write_string_LCD(char* str);
void move_cursor_first_row(int col);
void move_cursor_second_row(int col);
void clear_LCD(int mc);

#endif	/* SPI_H */

