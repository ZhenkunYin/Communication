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

static const message_type start_up_message={
		.mb_data = {0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00},
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
	QueueHandle_t que = (QueueHandle_t) flexcanState->callbackParam;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(eventType == FLEXCAN_EVENT_DMA_COMPLETE){
		if(recvMsg0.msgId == 0x18C)
		{
			monitor_list.RES = true;
			uint8_t tmp_message = recvMsg0.data[1];
			if (current_state != tmp_message){
				current_state = tmp_message;
				if(tmp_message == 0x00){
					/*emergency*/
					xQueueSendToBackFromISR(que,"E",xHigherPriorityTaskWoken);
					LPUART_DRV_SendData(INST_LPUART1,"emergency\n",12);
				}else if(tmp_message == 0x03){
					/*driving*/
					xQueueSendToBackFromISR(que,"D",xHigherPriorityTaskWoken);
					LPUART_DRV_SendData(INST_LPUART1,"start up\n",10);
				}else if(tmp_message == 0x01){
					/*ready*/
					xQueueSendToBackFromISR(que,"R",xHigherPriorityTaskWoken);
					LPUART_DRV_SendData(INST_LPUART1,"ready\n",7);
				}
			}
		}
		else if(recvMsg0.msgId == 0x501 || recvMsg0.msgId == 0x500){
			monitor_list.Jetson = true;
		}
#ifdef MOTOR
		else if(recvMsg0.msgId == )
		{

		}
#endif
		else if(recvMsg0.msgId == 0x70C)
		{
			monitor_list.RES = true;
			LPUART_DRV_SendData(INST_LPUART1,"init\n",6);
			FLEXCAN_DRV_Send(INST_CANCOM0, 9U, &data_info, start_up_message.msg_id, start_up_message.mb_data);
		}
		FLEXCAN_DRV_RxFifo(INST_CANCOM0,&recvMsg0);
	}
}

/**
 * @brief CANbus initialization
 */
static void flexcan_init(QueueHandle_t que){
	FLEXCAN_DRV_Init(INST_CANCOM0, &canCom0_State, &canCom0_InitConfig0);

	for(int i=0; i<2; i++){
		id_filter_table[i].isRemoteFrame = false;      /*!< Remote frame*/
		id_filter_table[i].isExtendedFrame = false;    /*!< Extended frame*/
	}

	id_filter_table[0].id = 0x18C;
	id_filter_table[1].id = 0x70C;

	FLEXCAN_DRV_ConfigRxFifo(INST_CANCOM0,FLEXCAN_RX_FIFO_ID_FORMAT_A,id_filter_table);
	FLEXCAN_DRV_InstallEventCallback(INST_CANCOM0,can_callback,que);
	EDMA_DRV_Init(&dmaController1_State, &dmaController1_InitConfig0, edmaChnStateArray, edmaChnConfigArray, EDMA_CONFIGURED_CHANNELS_COUNT);
	//FLEXCAN_DRV_SetRxMbGlobalMask(INST_CANCOM0,FLEXCAN_MSG_ID_STD,0xFFF);
	//FLEXCAN_DRV_RxFifo(INST_CANCOM0,&recvMsg0);
}

/**TASK************************************************************************
 * @brief messages sending task
 */
static void message_sending_task(void *pvParameters){
	(void) pvParameters;

	const TickType_t flexcan_task_delay = pdMS_TO_TICKS(20UL);

	while(1){

		vTaskDelay(flexcan_task_delay);
	}
}


/**TASK************************************************************************
 * @brief task content
 */
void message_monitoring_task(void *pvParameters){
	(void) pvParameters;

	const TickType_t flexcan_task_delay = pdMS_TO_TICKS(50UL);

	status_t status;

	while(1){
		status = FLEXCAN_DRV_RxFifo(INST_CANCOM0,&recvMsg0);

		if(status == STATUS_SUCCESS){

		}

		if(test){
			test = false;
			FLEXCAN_DRV_Send(INST_CANCOM0, 9U, &data_info, system_status.msg_id, system_status.mb_data);
		}

		vTaskDelay(flexcan_task_delay);
	}
}

/**
 * @brief setup flexcan task
 */
void FlexCAN_task_setup(QueueHandle_t que){
	flexcan_init(que);

	xTaskCreate(message_monitoring_task,
	    		"message monitor",
				configMINIMAL_STACK_SIZE,
				NULL,
				MONITORING_TASK_PRIORITY,
				NULL);

	/*xTaskCreate(message_sending_task,
		    	"CANbus sending",
				configMINIMAL_STACK_SIZE,
				NULL,
				CANBUS_SENDING_TASK_PRIORITY,
				NULL);*/
}
