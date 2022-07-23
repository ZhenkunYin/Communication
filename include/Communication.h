/*
 * Communication.h
 *

 * Author: Zhenkun
 */

/*kernel includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "s32k148.h"
#include "stdint.h"
#include "flexcan_driver.h"
#include "lpuart1.h"
#include "Cpu.h"

#ifndef MESSAGES_H_
#define MESSAGES_H_

#define FLEXCAN_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define UART_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define MESSAGES_PROCESS_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )

#define MY_TIME_OUT (100U)

typedef struct {
	uint32_t msg_id;
	uint8_t* mb_data;
	uint32_t timeout_ms;
}message_type;

/**flexcan.c***********************************************************
 *
 * @brief Functions for Flexcan
 */
void FlexCAN_task_setup();

/**UART.c**************************************************************
 *
 * @brief Functions for UART
 */
void UART_task_setup();


/**Message.c***********************************************************
 *
 * @brief Function for messages.c
 */

/*!
 * @brief Setup message task for processing received data from UART
 */
void Messages_task_setup();

/*!
 * @brief Interface for get specific data
 */
message_type get_driving_dynamics_1();

message_type get_driving_dynamics_2();

message_type get_system_status();

//void message_init();

void set_Speed_actual(uint8_t data);

uint8_t get_Speed_actual();

void set_Speed_target(uint8_t data);

uint8_t get_Speed_target();

void set_Steering_angle_actual(uint8_t data);

uint8_t get_Steering_angle_actual();

void set_Steering_angle_target(uint8_t data);

uint8_t get_Steering_angle_target();

void set_Brake_hydr_actual(uint8_t data);

uint8_t get_Brake_hydr_actual();

void set_Brake_hydr_target(uint8_t data);

uint8_t get_Brake_hydr_target();

void set_Motor_moment_actual(uint8_t data);

uint8_t get_Motor_moment_actual();

void set_Motor_moment_target(uint8_t data);

uint8_t get_Motor_moment_target();

void set_Acceleration_longitudinal(uint16_t data);

uint16_t get_Acceleration_longitudinal();

void set_Acceleration_lateral(uint16_t data);

uint16_t get_Acceleration_lateral();

void set_Yaw_rate(uint16_t data);

uint16_t get_Yaw_rate();

/**
 * @brief set AS state
 *
 * @param	state	1: ready
 * 					2: off
 * 					3: driving
 * 					4: emergency brake
 * 					5: finish
 */
void set_AS_state(uint8_t state);

uint8_t get_AS_state();

/**
 * @brief set EBS state
 *
 * @param	state	1: unavaliable
 * 					2: armed
 * 					3: activated
 */
void set_EBS_state(uint8_t state);

uint8_t get_EBS_state();

/**
 * @brief set AMI state
 *
 * @param	state	1: acceleration
 * 					2: skidpad
 * 					3: trackdrive
 * 					4: braketest
 * 					5: inspection
 * 					6: autocross
 */
void set_AMI_state(uint8_t state);

uint8_t get_AMI_state();

void set_Steering_state(bool state);

bool get_Steering_state();

/**
 * @brief set service brake state
 *
 * @param	state	1: disengaged
 * 					2: engaged
 * 					3: avaliable
 */
void set_Service_brake_state(uint8_t state);

uint8_t get_Service_brake_state();

void set_Lap_counter(uint8_t data);

uint8_t get_Lap_counter();

void set_Cones_count_actual(uint8_t data);

uint8_t get_Cones_count_actual();

void set_Cones_count_all(uint32_t data);

uint32_t get_Cones_count_all();

#endif /* MESSAGES_H_ */
