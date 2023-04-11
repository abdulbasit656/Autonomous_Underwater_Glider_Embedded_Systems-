#include "uart.h"
#include "circularBuffer.h"

volatile circular_buffer_t *_inBuffer;                      // The buffer used for reading from UART
volatile circular_buffer_t *_outBuffer;                     // The buffer used for writing to UART

void _uart_to_buffer();
void _buffer_to_uart();

// Function to setup the UART, with an input and output buffers
void UART_setup(int baudrate, volatile circular_buffer_t *inBuffer, volatile circular_buffer_t *outBuffer){
    //UART2
    
    long int BRGVAL = ((FCY/baudrate)/16)-1;        // ( Fosc / 4) / (16 * Baud Rate)-1
    U2BRG = BRGVAL;
    
    U2STAbits.URXISEL = 0b10;       // set the receiver interrupt when buffer is 3/4 full
    U2STAbits.UTXISEL = 1;          // set the transmitter interrupt when buffer is empty
    
    IFS1bits.U2TXIF = 0;            // Clear TX Interrupt flag
    IFS1bits.U2RXIF = 0;            // Clear TX Interrupt flag
    IEC1bits.U2RXIE = 1;            // enable UART receiver interrupt
    IEC1bits.U2TXIE = 1;            // enable UART transmitter interrupt
    
    U2MODEbits.UARTEN = 1;          // enable UART 
    U2STAbits.UTXEN = 1;            // enable UART TX (must be after UARTEN)
    
    // Storing the buffers
    _inBuffer = inBuffer;
    _outBuffer = outBuffer;
}

//interrupt to receive data from UART receiver
void __attribute__ (( __interrupt__ , __auto_psv__ ) ) _U2RXInterrupt() 
{
    IFS1bits.U2RXIF = 0;                // Reset the interrupt flag
    _uart_to_buffer();                  // Handle the reading from the buffer
}

// This is triggered when the transmitter UART buffer becomes empty
void __attribute__((__interrupt__, __auto_psv__)) _U2TXInterrupt()
{ 
    IFS1bits.U2TXIF = 0;                // Reset the interrupt flag
    _buffer_to_uart();           // Handle the writing on the buffer
}

//read data from UART to circular buffer
void UARTget() {
    
    IEC1bits.U2RXIE = 0;                                //disable the UART interrupt to read data
    _uart_to_buffer();                                  //reading from the buffer
    IEC1bits.U2RXIE = 1;                                // Re-enable UART interrupt again
    
    if(U2STAbits.OERR == 0)                             // Check if there was an overflow in the UART buffer     
        return;
    U2STAbits.OERR = 0;                                 //clearing the UART overflow flag.
}

//write a msg on the out circuar buffer
void UARTsend(char* msg) 
{
    IEC1bits.U2TXIE = 0;                               //disabling the interrupt
    
    if (write_cb_string(_outBuffer, msg)== -1)          //store msg in buffer
        return;

    _buffer_to_uart();                                  // write the whole string on the UART
    IEC1bits.U2TXIE = 1;                                    // Re-enabling the interrupt
}

// Function to read from UART
void _uart_to_buffer()
{
    while(U2STAbits.URXDA == 1)                         //read from UART and then fill the input buffer
        write_cb(_inBuffer, U2RXREG);
}

// Function to write on UART
void _buffer_to_uart()
{
    char character;
    while (U2STAbits.UTXBF == 0)
    {
        if(!read_cb(_outBuffer, &character))
            break;
        U2TXREG = character;
    }
    
}


