#include"circularBuffer.h"

void Buffer_setup(volatile circular_buffer_t *cb, char* arr, int size) {
    cb->buffer = arr;
    cb->size = size;
    cb->readIndex = 0;
    cb->writeIndex = 0;
}
//write in buffer
void write_cb(volatile circular_buffer_t* cb, char byte) {
    cb->buffer[cb->writeIndex] = byte;
    cb->writeIndex = (cb->writeIndex + 1) % cb->size;
    if (cb->readIndex == cb->writeIndex) {
        // full buffer
        cb->readIndex++; // discard the oldest byte
    }
    //overflow check
    if (cb->writeIndex == cb->size)
        cb->writeIndex = 0;
}

//add a string to the buffer
int write_cb_string(volatile circular_buffer_t *cb, char* string)
{
    int length = 0;

    while(string[length++] != '\0');
    --length;
    if(length > (cb->size)-(cb->readIndex))
        return -1;  // Not enough space for the string
    // Inserting the whole string
    for(int i=0; i<length; ++i)
    {
        cb->buffer[cb->writeIndex] = string[i];
        cb->writeIndex = (cb->writeIndex+1) % cb->size;
    }
    cb->readIndex += length;
    return 0;
}

//read from buffer
int read_cb(volatile circular_buffer_t* cb, char* byte) {
    //IEC1bits.U2RXIE = 0; // Disable interrupt of UART
    if (cb->readIndex == cb->writeIndex) { // We've finished reading
        IEC1bits.U2RXIE = 1; // Enable interrupt of UART
        return 0;
    }
    
    if (cb->readIndex != cb->writeIndex) {
        *byte = cb->buffer[cb->readIndex];
        cb->readIndex = (cb->readIndex + 1) % cb->size;
        
    }
    if(cb->readIndex == cb->size){
        cb->readIndex = 0;
    }
    return 1;
    //IEC1bits.U2RXIE = 1; // Enable interrupt of UART
}


int bufSize(volatile circular_buffer_t* cb) {
    if (cb->readIndex <= cb->writeIndex) {
        return cb->writeIndex - cb->readIndex;
    } else {
        return cb->size - cb->readIndex + cb->writeIndex;
    }
}

