/* Include Prototypes for your library, if applicable */

#include "mylibrary.h"

void heart_beat(){
	//after find fault, record id and send SIGNAL()
}

void can_init(){
	//initial hardware for canbus and prepare for sending and recieving
	//start heart beats for fault detection
	heart_beat();
}

void send_message(MESSAGE send_info, const int controller_id){
	//send to the right place
}

int receive_message(MESSAGE *receive_info){
	int id = receive_info->listener_id;

	/* waiting until receiving message
	 * set received message into recieve_info
	 * and return 1
	 * otherwise, return 0
	 * */


	return 0;
}

    /* to avoid the warning message for GHS: a translation unit must contain at least one declaration */
#if defined (__ghs__)
#pragma ghs nowarning 96
#endif
