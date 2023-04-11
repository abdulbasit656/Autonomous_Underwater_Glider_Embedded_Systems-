#include "xc.h"
#include "timer.h"
#include "button.h"

// Function enabling flag of buttons
void Button_setup(){
    IEC0bits.T3IE = 1;              // Enable interrupt of debouncing timer t3
    IFS1bits.INT1IF = 0;            // Reset interrupt flag for S6 button
    IEC1bits.INT1IE = 1;            // Enable interrupt for S6 button
}

void LedPins_setup()
{
    // Set the led pins as output
    TRISBbits.TRISB0 = 0;   // D3   
    TRISBbits.TRISB1 = 0;   // D4
}