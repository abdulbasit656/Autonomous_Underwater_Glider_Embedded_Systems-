#ifndef PARSER_H
#define	PARSER_H

#include <xc.h> // include processor files - each processor file is guarded.

#define STATE_DOLLAR  (1) // we discard everything until a dollar is found
#define STATE_TYPE    (2) // we are reading the type of msg until a comma is found
#define STATE_PAYLOAD (3) // we read the payload until an asterix is found
#define NEW_MESSAGE (1) // new message received and parsed completely
#define NO_MESSAGE (0) // no new messages

typedef struct { 
	int state;
	char msg_type[9]; // $ + msgtype is 5 chars + string terminator
	char msg_payload[50];  // assume payload cannot be longer than 25 chars
	int index_type;
	int index_payload;
} parser_state;

int parse_byte(parser_state* ps, char byte);
int extract_integer(const char* str, int* n);
int extract_float(const char* str, float* n);
int next_value(const char* msg, int i);
//int parse_rlsen(const char* msg);

#endif	/* PARSER_H */

