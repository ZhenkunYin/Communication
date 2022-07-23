/*
 * UART.c
 *
 *  Created on: 2022Äê7ÔÂ13ÈÕ
 *      Author: 24032
 */
#include "Communication.h"

static uint8_t uart_rec_buf[256];

extern QueueHandle_t que;

/**
 * @brief call back function for UART interruption
 */
static void uart_callback(void *driverState, uart_event_t event, void *userData){
	(void) userData;
	(void) driverState;
	LPUART_DRV_ReceiveData(INST_LPUART1,uart_rec_buf,256);
	if(event == UART_EVENT_RX_FULL){
		LPUART_DRV_SendData(INST_LPUART1,"this is interrupt\n",20);
		//xQueueSend(que,(void *)uart_rec_buf,100);

	}
}

/**
 * @brief Initialization step
 */
static void UART_init(){
	LPUART_DRV_Init(INST_LPUART1, &lpuart1_State, &lpuart1_InitConfig0);
	LPUART_DRV_InstallRxCallback(INST_LPUART1,uart_callback,NULL);
}


bool test = false;

/**
 * @brief Main function for UART
 */
static void UART_recieve_task(void *pvParameters){
	(void) pvParameters;

	const TickType_t UART_task_delay = pdMS_TO_TICKS(500UL);
	LPUART_DRV_SendData(INST_LPUART1,"UART init",10);

	while(1){
		//LPUART_DRV_ReceiveDataBlocking(INST_LPUART1,uart_rec_buf,256,100);
		//if(strstr(uart_rec_buf,"11\n")){
			LPUART_DRV_SendData(INST_LPUART1,"this is test\n",14);
		//}
		//memset(uart_rec_buf,0,256);
		//LPUART_DRV_SendData(INST_LPUART1,uart_rec_buf,256);
		//LPUART_DRV_SendData(INST_LPUART1,"hello UART\n",11);
		vTaskDelay(UART_task_delay);
	}

}

/**
 * @brief Setup UART task
 */
void UART_task_setup(){
	UART_init();

	xTaskCreate(UART_recieve_task,
	        		"UART",
	    			configMINIMAL_STACK_SIZE,
	    			NULL,
					0,//UART_RECEIVE_TASK_PRIORITY,
	    			NULL);
}
