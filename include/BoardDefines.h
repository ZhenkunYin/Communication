/*
 * BoardDefines.h
 *
 *  Created on: Sep 1, 2022
 *      Author: 24032
 */

#ifndef BOARDDEFINES_H_
#define BOARDDEFINES_H_

#include <stdint.h>
#include <stdbool.h>
#include "Communication.h"

#define FSM_TASK_PRIORITY 					( tskIDLE_PRIORITY + 0 )

#define EVB


    #define LED1            22U
    #define LED2            21U
	#define LED3 			23U
	#define LED4			27U
	#define LED5			26U
    #define LED_GPIO        PTE
    #define LED_PORT        PORTE
    #define LED_PORT_PCC    PCC_PORTE_CLOCK
    #define BTN_GPIO        PTC
    #define BTN_PIN         13U
 	#define BTN_PIN1        12U
    #define BTN_PORT        PORTC
    #define BTN_PORT_PCC    PCC_PORTC_CLOCK
    #define BTN_PORT_IRQn   PORTC_IRQn


#include "pins_driver.h"



typedef enum {
  START,
  READY,
  DRIVING,
  FINISHED,
  STOP,
  EMERGENCY
} FSM_STATES;

typedef struct {
  FSM_STATES state;
  char sig;
  FSM_STATES next_state;
} FSM_STATE_INFO_t;


void FSM_task_setup();

#endif /* BOARDDEFINES_H_ */
