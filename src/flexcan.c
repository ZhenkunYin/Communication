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

static flexcan_data_info_t data_info = {
	.msg_id_type = FLEXCAN_MSG_ID_STD,  /*!< Type of message ID (standard or extended)*/
	.data_length = 8U,                   /*!< Length of Data in Bytes*/
	.is_remote = false
};

static flexcan_msgbuff_t recvMsg0;		//FlexCAN receive buffer

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
	(void) flexcanState;

	if(eventType == FLEXCAN_EVENT_RX_COMPLETE) {
		if(instance == INST_CANCOM0) { //Receive data from can0 into recvmsg0, and send it

			FLEXCAN_DRV_Receive(INST_CANCOM0, RX_MAILBOX_CAN0, &recvMsg0);
			recvMsg0.data[1]=0xFF;
			FLEXCAN_DRV_Send(INST_CANCOM0, TX_MAILBOX_CAN0, &data_info, recvMsg0.msgId, recvMsg0.data);
		}
	}else if(eventType == FLEXCAN_EVENT_DMA_COMPLETE){
		if(recvMsg0.data[0] == 0x03){
			recvMsg0.data[1] == 0xCC;
		}
		FLEXCAN_DRV_Send(INST_CANCOM0, TX_MAILBOX_CAN0, &data_info, recvMsg0.msgId, recvMsg0.data);
		FLEXCAN_DRV_RxFifo(INST_CANCOM0,&recvMsg0);
	}
}

/**
 * @brief CANbus initialization
 */
static void flexcan_init(){
	FLEXCAN_DRV_Init(INST_CANCOM0, &canCom0_State, &canCom0_InitConfig0);

	flexcan_id_table_t id_filter_table = {
		.isRemoteFrame = false,      /*!< Remote frame*/
		.isExtendedFrame = false,    /*!< Extended frame*/
		.id = 0x18C
	};

	//FLEXCAN_DRV_ConfigRxFifo(INST_CANCOM0,FLEXCAN_RX_FIFO_ID_FORMAT_A,&id_filter_table);

	FLEXCAN_DRV_ConfigRxMb(INST_CANCOM0,RX_MAILBOX_CAN0,&data_info,0x500);

	FLEXCAN_DRV_InstallEventCallback(INST_CANCOM0,can_callback,NULL);

	//FLEXCAN_DRV_SetRxMbGlobalMask(INST_CANCOM0,FLEXCAN_MSG_ID_STD,0xFFF);
	//FLEXCAN_DRV_RxFifo(INST_CANCOM0,&recvMsg0);
}

/**
 * @brief sending messages
 *
 * @param	message: message data, ID, timeout
 */
static void send_messages(const message_type message){
	FLEXCAN_DRV_Send(INST_CANCOM0,TX_MAILBOX_CAN0,&data_info,message.msg_id,message.mb_data);
}


extern bool test;

/**
 * @brief task content
 */
void message_sending_task(void *pvParameters){
	(void) pvParameters;

	const TickType_t flexcan_task_delay = pdMS_TO_TICKS(1000UL);

	while(1){
		/*if(FLEXCAN_DRV_ReceiveBlocking(INST_CANCOM0, RX_MAILBOX_CAN0, &recvMsg0,100) == STATUS_SUCCESS){
			recvMsg0.data[1] = 0xFF;
			FLEXCAN_DRV_Send(INST_CANCOM0,TX_MAILBOX_CAN0,&data_info,recvMsg0.msgId,recvMsg0.data);
		}*/
		if(test){
			test=false;
			send_messages(driving_dynamics_1);
		}

		//driving_dynamics_1.mb_data[0]++;
		//send_messages(driving_dynamics_1);
		vTaskDelay(flexcan_task_delay);
	}
}

/**
 * @brief setup flexcan task
 */
void FlexCAN_task_setup(){
	flexcan_init();

	xTaskCreate(message_sending_task,
	    		"Flexcan",
				configMINIMAL_STACK_SIZE,
				NULL,
				FLEXCAN_TASK_PRIORITY,
				NULL);
}
