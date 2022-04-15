/* Prototypes for your library, if applicable */

#include<stdio.h>
#include<string.h>

#ifndef CAN_PROTOCOL
#define CAN_PROTOCOL
//ID
#define NXP
#define RES
#define TUM

#endif

typedef struct{
	int buffer;
	int talker_id;
	int listener_id;
}MESSAGE;

/*Initialize CANbus state before running code*/
void can_init(){}

/*send message*/
void send_message(MESSAGE send_info, const int controller_id){}

/*recieve message*/
int receive_message(MESSAGE *receive_info){}
