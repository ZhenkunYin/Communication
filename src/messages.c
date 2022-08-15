/*
 * messages.c
 *
 * Author: Zhenkun
 */

#include "Communication.h"

uint8_t driving_dynamic_1_data[8] = {0,0,0,0,0,0,0,0};
uint8_t driving_dynamic_2_data[8]= {0,0,0,0,0,0,0,0};
uint8_t system_status_data[8] = {0,0,0,0,0,0,0,0};

extern QueueHandle_t que;

static uint8_t *data;

/*
static void messages_process_task(){
	while(1){
		xQueueReceive(que,(void *)data,0);
	}
}*/

void Messages_task_setup(){
	/*xTaskCreate(messages_process_task,
		    		"Messages",
					configMINIMAL_STACK_SIZE,
					NULL,
					MESSAGES_PROCESS_TASK_PRIORITY,
					NULL);*/
}
#ifndef JETSON
#define JETSON
message_type driving_dynamics_1 = {
		.msg_id = 0x500,
		.mb_data = driving_dynamic_1_data,
		.timeout_ms = MY_TIME_OUT
};

message_type driving_dynamics_2 = {
		.msg_id = 0x501,
		.mb_data = driving_dynamic_2_data,
		.timeout_ms = MY_TIME_OUT
};
#endif

#ifndef RES_T
#define RES_T
message_type system_status = {
		.msg_id = 0x502,
		.mb_data = system_status_data,
		.timeout_ms = MY_TIME_OUT
};
#endif

message_type get_driving_dynamics_1(){
	return driving_dynamics_1;
}

message_type get_driving_dynamics_2(){
	return driving_dynamics_2;
}

message_type get_system_status(){
	return system_status;
}


void set_Speed_actual(uint8_t data){
	driving_dynamics_1.mb_data[0] = data;
}

uint8_t get_Speed_actual(){
	return driving_dynamics_1.mb_data[0];
}

void set_Speed_target(uint8_t data){
	driving_dynamics_1.mb_data[1] = data;
}

uint8_t get_Speed_target(){
	return driving_dynamics_1.mb_data[1];
}

void set_Steering_angle_actual(uint8_t data){
	driving_dynamics_1.mb_data[2] = data;
}

uint8_t get_Steering_angle_actual(){
	return driving_dynamics_1.mb_data[2];
}

void set_Steering_angle_target(uint8_t data){
	driving_dynamics_1.mb_data[3] = data;
}

uint8_t get_Steering_angle_target(){
	return driving_dynamics_1.mb_data[3];
}

void set_Brake_hydr_actual(uint8_t data){
	driving_dynamics_1.mb_data[4] = data;
}

uint8_t get_Brake_hydr_actual(){
	return driving_dynamics_1.mb_data[4];
}

void set_Brake_hydr_target(uint8_t data){
	driving_dynamics_1.mb_data[5] = data;
}

uint8_t get_Brake_hydr_target(){
	return driving_dynamics_1.mb_data[5];
}

void set_Motor_moment_actual(uint8_t data){
	driving_dynamics_1.mb_data[6] = data;
}

uint8_t get_Motor_moment_actual(){
	return driving_dynamics_1.mb_data[6];
}

void set_Motor_moment_target(uint8_t data){
	driving_dynamics_1.mb_data[7] = data;
}

uint8_t get_Motor_moment_target(){
	return driving_dynamics_1.mb_data[7];
}

void set_Acceleration_longitudinal(uint16_t data){
	driving_dynamics_2.mb_data[0] = data & 0xff;
	driving_dynamics_2.mb_data[1] = (data & 0xff00) >> 8;
}

uint16_t get_Acceleration_longitudinal(){
	return (driving_dynamics_2.mb_data[0] & 0xff) | ((driving_dynamics_2.mb_data[1] & 0xff)<<8);
}

void set_Acceleration_lateral(uint16_t data){
	driving_dynamics_2.mb_data[2] = data & 0xff;
	driving_dynamics_2.mb_data[3] = (data & 0xff00) >> 8;
}

uint16_t get_Acceleration_lateral(){
	return (driving_dynamics_2.mb_data[2] & 0xff) | ((driving_dynamics_2.mb_data[3] & 0xff) << 8);
}

void set_Yaw_rate(uint16_t data){
	driving_dynamics_2.mb_data[4] = data & 0xff;
	driving_dynamics_2.mb_data[5] = (data & 0xff00) >> 8;
}

uint16_t get_Yaw_rate(){
	return (driving_dynamics_2.mb_data[4] & 0xff) | ((driving_dynamics_2.mb_data[5] & 0xff) << 8);
}

/**
 * @brief set AS state
 */
void set_AS_state(uint8_t state){
	system_status.mb_data[0] = (system_status.mb_data[0] & (~0x07))|(state << 0);
}

uint8_t get_AS_state(){
	return system_status.mb_data[0] & 0x07;
}

/**
 * @brief set EBS state
 */
void set_EBS_state(uint8_t state){
	system_status.mb_data[0] = (system_status.mb_data[0] & (~(0x03<<3)))|(state << 3);
}

uint8_t get_EBS_state(){
	return (system_status.mb_data[0] >> 3) & 0x03;
}

/**
 * @brief set AMI state
 */
void set_AMI_state(uint8_t state){
	system_status.mb_data[0] = (system_status.mb_data[0] & (~(0x07<<5)))|(state << 5);
}

uint8_t get_AMI_state(){
	return (system_status.mb_data[0] >> 5) & 0x07;
}

void set_Steering_state(bool state){
	if(state)
		system_status.mb_data[1] |= 0x01;
	else
		system_status.mb_data[1] &= (~0x01);
}

bool get_Steering_state(){
	return system_status.mb_data[1] & 0x01;
}

/**
 * @brief set service brake state
 */
void set_Service_brake_state(uint8_t state){
	system_status.mb_data[1] = (system_status.mb_data[1] & (~(0x03<<1)))|(state << 1);
}

uint8_t get_Service_brake_state(){
	return (system_status.mb_data[1] >> 1) & 0x03;
}

void set_Lap_counter(uint8_t data){
	system_status.mb_data[1] = (system_status.mb_data[1] & (~(0x0f<<3)))|(data << 3);
}

uint8_t get_Lap_counter(){
	return (system_status.mb_data[1] >> 3) & 0x0f;
}

void set_Cones_count_actual(uint8_t data){
	system_status.mb_data[1] = (system_status.mb_data[1] & (~(0x01<<7)))|((data & 0x01) << 7);
	system_status.mb_data[2] = (system_status.mb_data[2] & (~0x7f))|(data >> 1);
}

uint8_t get_Cones_count_actual(){
	return ((system_status.mb_data[1] & 0xff) >> 7) | ((system_status.mb_data[2] & 0x7f) << 1);
}

void set_Cones_count_all(uint32_t data){
	system_status.mb_data[2] = (system_status.mb_data[2] & (~(0x01<<7)))|((data & 0x01)<<7);
	system_status.mb_data[3] &= ((data >> 1) & 0xff);
	system_status.mb_data[3] &= (data >> 9);
}

uint32_t get_Cones_count_all(){
	return ((system_status.mb_data[2] & 0xff) >> 7) | ((system_status.mb_data[3] & 0xff) << 1) | ((system_status.mb_data[4] & 0xff) << 9);
}



