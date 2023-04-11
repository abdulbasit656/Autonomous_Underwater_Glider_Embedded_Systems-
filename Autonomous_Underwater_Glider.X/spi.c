#include "spi.h"

//LCD setup
void SPI_setup(){           //LCD communication register configuration
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8?bit mode
    SPI1CONbits.PPRE = 0b11; // 1:1 primary prescaler
    SPI1CONbits.SPRE = 0b110; // 5:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI
}


void put_char_SPI(char c) {
    while (SPI1STATbits.SPITBF == 1); // wait for previous transmissions to finish
    SPI1BUF = c;
}

void write_string_LCD(char* str) {
    int i = 0;
    for (i = 0; str[i] != '\0'; i++) {
        put_char_SPI(str[i]);
    }
}

void move_cursor_first_row(int col) {
    put_char_SPI(0x80 + col);
}

void move_cursor_second_row(int col) {
    put_char_SPI(0xC0 + col);
}

//clear lcd ROW by given row number
void clear_LCD(int mc) {
    switch(mc){
        case FirstRow:{
                move_cursor_first_row(0);
                int i = 0;
                for (i = 0; i < 16; i++) {
                    put_char_SPI(' '); // write spaces to ?clear? the LCD from previous characters    
                }
                break;
            }
        case SecRow:
        {
            move_cursor_second_row(0);
            int i = 0;
            for (i = 0; i < 16; i++) {
                put_char_SPI(' '); // write spaces to ?clear? the LCD from previous characters
            }
            break;
        }
    }
}

