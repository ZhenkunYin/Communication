/*
 * flexcan.c
 *
 * Author: Zhenkun
 *
 * @brief file to set message data and transceive the data
 */

#include "Communication.h"

#define RX_MAILBOX_CAN0  (0UL)
#define TX_MAILBOX_CAN0  (1UL)

/**
 * @brief global data
 */
bool test = false;
uint8_t RES_state_data;
volatile char FSM_state;
bool start_state = false;

message_check_list monitor_list = {
		.RES = false,
#ifdef MOTOR
		.motor_controller = false,
#else
		.motor_controller = true,
#endif
		.Jetson = false
};

/**
 * @brief private data
 */
static flexcan_data_info_t data_info = {
	.msg_id_type = FLEXCAN_MSG_ID_STD,  /*!< Type of message ID (standard or extended)*/
	.data_length = 8U,                   /*!< Length of Data in Bytes*/
	.is_remote = false
};

static const uint8_t start_up_message_data[8] = {0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
static message_type start_up_message = {
		.mb_data = start_up_message_data,
		.msg_id = 0x00,
		.timeout_ms = 10
};

static flexcan_msgbuff_t recvMsg0;		//FlexCAN receive buffer
static uint8_t current_state;
static flexcan_id_table_t id_filter_table[8];

/**
 * @brief external global data
 */
extern message_type driving_dynamics_1;
extern message_type driving_dynamics_2;
extern message_type system_status;
extern uint8_t UART_system_state[10];
extern uint8_t system_status_data[8];
/**
 * @brief call back events
 *
 * Currently for receive events. and it will keep sending the received data
 *
 *@param	instance: Device instance
 *@param	eventType: Interrupt event
 *@param	buffIdx:
 *@param	flexcanState: Internal driver state information
 */
void can_callback(uint8_t instance, flexcan_event_type_t eventType,
		uint32_t buffIdx, flexcan_state_t *flexcanState) {
	(void) buffIdx;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(eventType == FLEXCAN_EVENT_DMA_COMPLETE){
		if(recvMsg0.msgId == 0x18C)
		{
			monitor_list.RES = true;
			RES_state_data = recvMsg0.data[1];
#if CAN_REV_TEST
			uint8_t tmp_message = recvMsg0.data[1];
			uint8_t state = 0x00;

			if (current_state != tmp_message){
				current_state = tmp_message;
				if(tmp_message == 0x00){
					/*emergency*/
					state = 0x04;
					//LPUART_DRV_SendData(INST_LPUART1,"emergency\n",12);
				}else if(tmp_message == 0x03){
					/*driving*/
					state = 0x03;
					//LPUART_DRV_SendData(INST_LPUART1,"start up\n",10);
				}else if(tmp_message == 0x01){
					/*ready*/
					state = 0x02;
					//LPUART_DRV_SendData(INST_LPUART1,"ready\n",7);
				}
				set_AS_state(state);
				LPUART_DRV_SendData(INST_LPUART1,UART_system_state,10);
			}
#endif
		}
//		else if(recvMsg0.msgId == 0x501 || recvMsg0.msgId == 0x500){
//			monitor_list.Jetson = true;
//		}
#ifdef MOTOR
		else if(recvMsg0.msgId == 0x500)
		{
			strncpy(driving_dynamics_1,recvMsg0,8);
		}
		else if(recvMsg0.msgId == 0x501)
		{
			strncpy(driving_dynamics_2,recvMsg0,8);
		}
		else if(recvMsg0.msgId == 0x502)
		{
			strncpy(system_status,recvMsg0,8);
		}
#endif
		else if(recvMsg0.msgId == 0x70C)
		{
			monitor_list.RES = true;
			LPUART_DRV_SendData(INST_LPUART1,start_up_message_data,8);	//to Jetson, indicating that the system started
			FLEXCAN_DRV_Send(INST_CANCOM0, 9U, &data_info, start_up_message.msg_id, start_up_message.mb_data);	//send to RES
			start_state = true;

		}
		FLEXCAN_DRV_RxFifo(INST_CANCOM0,&recvMsg0);
	}
}

/**
 * @brief CANbus initialization
 */
static void flexcan_init(){
	FLEXCAN_DRV_Init(INST_CANCOM0, &canCom0_State, &canCom0_InitConfig0);

	for(int i=0; i<2; i++){
		id_filter_table[i].isRemoteFrame = false;      /*!< Remote frame*/
		id_filter_table[i].isExtendedFrame = false;    /*!< Extended frame*/
	}

	id_filter_table[0].id = 0x18C;
	id_filter_table[1].id = 0x70C;

	FLEXCAN_DRV_ConfigRxFifo(INST_CANCOM0,FLEXCAN_RX_FIFO_ID_FORMAT_A,id_filter_table);
	FLEXCAN_DRV_InstallEventCallback(INST_CANCOM0,can_callback,NULL);
	EDMA_DRV_Init(&dmaController1_State, &dmaController1_InitConfig0, edmaChnStateArray, edmaChnConfigArray, EDMA_CONFIGURED_CHANNELS_COUNT);
	//FLEXCAN_DRV_SetRxMbGlobalMask(INST_CANCOM0,FLEXCAN_MSG_ID_STD,0xFFF);
	//FLEXCAN_DRV_RxFifo(INST_CANCOM0,&recvMsg0);
}

/**TASK************************************************************************
 * @brief messages sending task
 */
static void message_sending_task(void *pvParameters){

	const TickType_t flexcan_task_delay = pdMS_TO_TICKS(50UL);
	TickType_t last_wake_time = xTaskGetTickCount();
	uint8_t buf_temp;
	while(1){
		//LPUART_DRV_SendData(INST_LPUART1, "twice\n", 5);
		FLEXCAN_DRV_RxFifo(INST_CANCOM0,&recvMsg0);
		if(start_state)
		{
			FLEXCAN_DRV_Send(INST_CANCOM0, 9U, &data_info, driving_dynamics_1.msg_id, driving_dynamics_1.mb_data);
			FLEXCAN_DRV_Send(INST_CANCOM0, 10U, &data_info, driving_dynamics_2.msg_id, driving_dynamics_2.mb_data);
			FLEXCAN_DRV_Send(INST_CANCOM0, 11U, &data_info, system_status.msg_id, system_status.mb_data);
		}

		if(system_status_data[0] == 0x04)
		{
			FSM_state = 'E';
		}
		else if(system_status_data[0] == 0x02)
		{
			FSM_state = 'R';
		}
		else if(system_status_data[0] == 0x03)
		{
			FSM_state = 'D';
		}

		vTaskDelayUntil(&last_wake_time,flexcan_task_delay);
	}
}


/**TASK************************************************************************
 * @brief task content
 */
void message_monitoring_task(void *pvParameters){
	(void) pvParameters;

	const TickType_t flexcan_task_delay = pdMS_TO_TICKS(50UL);
	TickType_t last_wake_time = xTaskGetTickCount();

	status_t status;

	while(1){
		status = FLEXCAN_DRV_RxFifo(INST_CANCOM0,&recvMsg0);

		if(status == STATUS_SUCCESS){

		}

		if(test){
			test = false;
			FLEXCAN_DRV_Send(INST_CANCOM0, 9U, &data_info, system_status.msg_id, system_status.mb_data);
		}
		//FLEXCAN_DRV_Send(INST_CANCOM0, 9U, &data_info, system_status.msg_id, system_status.mb_data);
		vTaskDelayUntil(&last_wake_time,flexcan_task_delay);
		//vTaskDelay(flexcan_task_delay);
	}
}

/**
 * @brief setup flexcan task
 */
void FlexCAN_task_setup(){
	flexcan_init();
	FSM_state = 'R';

	/*xTaskCreate(message_monitoring_task,
	    		"message monitor",
				configMINIMAL_STACK_SIZE,
				NULL,
				MONITORING_TASK_PRIORITY,
				NULL);*/

	xTaskCreate(message_sending_task,
		    	"CANbus sending",
				configMINIMAL_STACK_SIZE,
				NULL,
				CANBUS_SENDING_TASK_PRIORITY,
				NULL);
}
