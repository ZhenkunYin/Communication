/*
 * FSM.c
 *
 *  Created on: Sep 1, 2022
 *      Author: 24032
 */

#include "BoardDefines.h"

extern volatile char FSM_state;
static FSM_STATE_INFO_t FSM_STATE_INFO;

const char* stateNames[] = {"START","READY","DRIVING","FINISHED","STOP","EMERGENCY"};


void boardSetup(void)
{
    /* Configure ports */
    PINS_DRV_SetMuxModeSel(LED_PORT, LED1,      PORT_MUX_AS_GPIO);
    PINS_DRV_SetMuxModeSel(LED_PORT, LED2,      PORT_MUX_AS_GPIO);
    PINS_DRV_SetMuxModeSel(LED_PORT, LED3,      PORT_MUX_AS_GPIO);
    PINS_DRV_SetMuxModeSel(BTN_PORT, BTN_PIN,   PORT_MUX_AS_GPIO);
    PINS_DRV_SetMuxModeSel(BTN_PORT, BTN_PIN1,   PORT_MUX_AS_GPIO);


#ifdef EVB
    PINS_DRV_SetPinIntSel(BTN_PORT, BTN_PIN, PORT_INT_RISING_EDGE);
    PINS_DRV_SetPinIntSel(BTN_PORT, BTN_PIN1, PORT_INT_RISING_EDGE);
#else
    PINS_DRV_SetPinIntSel(BTN_PORT, BTN_PIN, PORT_INT_FALLING_EDGE);
#endif
}

/**
 * @brief setting up GPIOs
 */
static void prvSetupHardware( void )
{

    /* Initialize and configure clocks
     *  -   Setup system clocks, dividers
     *  -   see clock manager component for more details
     */
    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
                   g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);

    boardSetup();

    PINS_DRV_SetMuxModeSel(LED_PORT, LED3,      PORT_MUX_AS_GPIO);
    PINS_DRV_ClearPins(LED_GPIO, (1 << LED3));
    PINS_DRV_SetPins(LED_GPIO, (1 << LED3));


    PINS_DRV_SetMuxModeSel(LED_PORT, LED4,      PORT_MUX_AS_GPIO);
    PINS_DRV_ClearPins(LED_GPIO, (1 << LED4));
    PINS_DRV_SetPins(LED_GPIO, (1 << LED4));

	/* Change LED1, LED2 to outputs. */
	PINS_DRV_SetPinsDirection(LED_GPIO,  (1 << LED1) | (1 << LED2) | (1 << LED3) | (1 << LED4));

	/* Change BTN1 to input */
	PINS_DRV_SetPinsDirection(BTN_GPIO, ~((1 << BTN_PIN)|(1 << BTN_PIN1)) );
	//PINS_DRV_SetPinsDirection(BTN_GPIO, ~(1 << BTN_PIN1) );

	/* Start with LEDs off. */
	PINS_DRV_ClearPins(LED_GPIO, (1 << LED1) | (1 << LED2));

	/* Install Button interrupt handler */
    //INT_SYS_InstallHandler(BTN_PORT_IRQn, button_press, (isr_t *)NULL);

    INT_SYS_SetPriority( BTN_PORT_IRQn,3);
}

void update_state()
{
	if(FSM_state == 'E') FSM_STATE_INFO.state = EMERGENCY;
	else if(FSM_state == 'R') FSM_STATE_INFO.state = READY;
	else if(FSM_state == 'D') FSM_STATE_INFO.state = DRIVING;
}

void diplay_state(){
	if (FSM_STATE_INFO.state == READY){
		PINS_DRV_ClearPins(LED_GPIO, (1 << LED1)|(1 << LED2)|(1 << LED3));
		PINS_DRV_TogglePins(LED_GPIO, (1 << LED1));
	}
	else if (FSM_STATE_INFO.state == DRIVING){
		PINS_DRV_ClearPins(LED_GPIO, (1 << LED1)|(1 << LED2)|(1 << LED3));
		PINS_DRV_TogglePins(LED_GPIO, (1 << LED2));
	}
	else if (FSM_STATE_INFO.state == FINISHED){
		PINS_DRV_ClearPins(LED_GPIO, (1 << LED1)|(1 << LED2)|(1 << LED3));
		PINS_DRV_TogglePins(LED_GPIO, (1 << LED3));
	}
}

void FSM_task()
{
	prvSetupHardware();
	PINS_DRV_TogglePins(LED_GPIO, (1 << LED1));
	const TickType_t flexcan_task_delay = pdMS_TO_TICKS(50UL);
	TickType_t last_wake_time = xTaskGetTickCount();

	while(1)
	{
		update_state();
		diplay_state();
		vTaskDelayUntil(&last_wake_time,flexcan_task_delay);
	}
}

void FSM_task_setup()
{
	xTaskCreate(FSM_task,
		    	"FSM",
				configMINIMAL_STACK_SIZE,
				NULL,
				FSM_TASK_PRIORITY,
				NULL);
}
