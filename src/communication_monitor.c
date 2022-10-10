/*
 * communication_monitor.c
 *
 *  Created on: 2022Äê8ÔÂ14ÈÕ
 *      Author: 24032
 */

#include "communication.h"

extern message_check_list monitor_list;

static bool mcu_state = false;
static bool warning = false;
static const uint32_t LPIT_CHANNEL = 0;

/**
 * @brief warning state handle function
 */
void warning_handle_func(){
	warning = false;
	LPUART_DRV_SendData(INST_LPUART1," time out\n",9);
}

/**
 * @brief monitoring MCUs state
 */
void communication_monitor_task(void *pvParameters){
	(void) pvParameters;

	const TickType_t communication_monitor_task_delay = pdMS_TO_TICKS(100UL);

	while(1)
	{
		if(warning)
		{
			warning_handle_func();
			monitor_list.Jetson = false;
			monitor_list.RES = false;
#ifdef MOTOR
			monitor_list.motor_controller = false;
#endif
		}
		if(monitor_list.Jetson && monitor_list.RES && monitor_list.motor_controller)
		{
			mcu_state = true;
		}
		vTaskDelay(communication_monitor_task_delay);
	}
}

/**
 * @brief 200ms cycle for checking MCUs state
 */
void LPIT_ISR(void)
{
    /* Clear LPIT channel flag */
    LPIT_DRV_ClearInterruptFlagTimerChannels(INST_LPIT1, (1 << LPIT_CHANNEL));
    if(!mcu_state){
    	warning = true;
    }else{
    	monitor_list.Jetson = false;
    	monitor_list.RES = false;
#ifdef MOTOR
    	monitor_list.motor_controller = false;
#endif
    }
}

void communication_monitor_setup(){
	/* Initialize LPIT instance 0
	 *  -   Reset and enable peripheral
	*/
	LPIT_DRV_Init(INST_LPIT1, &lpit1_InitConfig);
	/* Initialize LPIT channel 0 and configure it as a periodic counter
	 * which is used to generate an interrupt every second.
	*/
	LPIT_DRV_InitChannel(INST_LPIT1, LPIT_CHANNEL, &lpit1_ChnConfig0);

	/* Install LPIT_ISR as LPIT interrupt handler */
	INT_SYS_InstallHandler(LPIT0_Ch0_IRQn, &LPIT_ISR, (isr_t *)0);

	/* Start LPIT0 channel 0 counter */
	LPIT_DRV_StartTimerChannels(INST_LPIT1, (1 << LPIT_CHANNEL));

	xTaskCreate(communication_monitor_task,
		    		"communication monitor",
					configMINIMAL_STACK_SIZE,
					NULL,
					MONITORING_TASK_PRIORITY,
					NULL);
}
