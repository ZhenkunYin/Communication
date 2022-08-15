/*
 * UART.c
 *
 *  Created on: 2022Äê7ÔÂ13ÈÕ
 *      Author: 24032
 */
#include "Communication.h"
#include "string.h"

#define errorMsg "An error occurred! The application will stop!\r\n"
#define BUFFER_SIZE 256U
#define TIMEOUT 10U

uint8_t bufferIdx;

uint8_t uart_rec_buf[BUFFER_SIZE];

extern QueueHandle_t que;
extern bool test;
extern uint8_t system_status_data[8];

/**
 * @brief call back function for UART interruption
 */
static void uart_callback(void *driverState, uart_event_t event, void *userData){
	(void) userData;
	(void) driverState;
	//LPUART_DRV_ReceiveData(INST_LPUART1,uart_rec_buf,256);
	if(event == UART_EVENT_RX_FULL){
		//LPUART_DRV_SendData(INST_LPUART1,"INT1",4);
		/* The reception stops when newline is received or the buffer is full */
		if (bufferIdx != 7U){
			//LPUART_DRV_SendData(INST_LPUART1,"INT2",4);
			/* Update the buffer index and the rx buffer */
			bufferIdx++;
			LPUART_DRV_SetRxBuffer(INST_LPUART1, &uart_rec_buf[bufferIdx], 1U);
		}else{
			strncpy(system_status_data,uart_rec_buf,sizeof(system_status_data));
			test = true;
		}

#if 0
		if ((uart_rec_buf[bufferIdx] != '\n') && (bufferIdx != (BUFFER_SIZE - 2U))){
			//LPUART_DRV_SendData(INST_LPUART1,"INT2",4);
			/* Update the buffer index and the rx buffer */
			bufferIdx++;
			LPUART_DRV_SetRxBuffer(INST_LPUART1, &uart_rec_buf[bufferIdx], 1U);
		}
#endif
	}
}

/**
 * @brief Initialization step
 */
static void UART_init(){
	LPUART_DRV_Init(INST_LPUART1, &lpuart1_State, &lpuart1_InitConfig0);
	LPUART_DRV_InstallRxCallback(INST_LPUART1,uart_callback,NULL);
}

/**
 * @brief Main function for UART
 */
static void UART_recieve_task(void *pvParameters){
	(void) pvParameters;
	uint32_t bytesRemaining;
	status_t status;

	const TickType_t UART_task_delay = pdMS_TO_TICKS(50UL);
	LPUART_DRV_SendData(INST_LPUART1,"UART init",10);

	while(1){
		LPUART_DRV_ReceiveData(INST_LPUART1,uart_rec_buf,1U);

		while(LPUART_DRV_GetReceiveStatus(INST_LPUART1,&bytesRemaining) == STATUS_BUSY);

		status = LPUART_DRV_GetReceiveStatus(INST_LPUART1, &bytesRemaining);

		if(status != STATUS_SUCCESS){
			LPUART_DRV_SendDataBlocking(INST_LPUART1, (uint8_t *)errorMsg, strlen(errorMsg), TIMEOUT);
			break;
		}

		bufferIdx++;
		uart_rec_buf[bufferIdx] = 0U;

	    LPUART_DRV_SendData(INST_LPUART1, uart_rec_buf, bufferIdx);

	    bufferIdx = 0U;

		//if(strstr(uart_rec_buf,"11\n")){
		//LPUART_DRV_SendData(INST_LPUART1,"this is test\n",14);
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
					UART_RECEIVE_TASK_PRIORITY,
	    			NULL);
}
