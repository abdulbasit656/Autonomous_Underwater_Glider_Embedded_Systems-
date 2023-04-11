
// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef CIRCULAR_BUFFER_H
#define	CIRCULAR_BUFFER_H

#include <xc.h> // include processor files - each processor file is guarded.  

typedef struct {
    char *buffer;
    int size;
    int readIndex;
    int writeIndex;
} circular_buffer_t;


void Buffer_setup(volatile circular_buffer_t *cb, char* arr, int size);                      // Function to init transmission buffer
void write_cb(volatile circular_buffer_t* cb, char byte);
int write_cb_string(volatile circular_buffer_t *cb, char* string);
int read_cb(volatile circular_buffer_t* cb, char* byte);
int bufSize(volatile circular_buffer_t* cb);

#endif	/* CIRCULAR_BUFFER_H */

