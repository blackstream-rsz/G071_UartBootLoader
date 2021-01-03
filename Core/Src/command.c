#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "NakedG071C8T6_Test.h"

#define hTargetUart					huart4
#define TARGET_USART					USART4

/* boot loader コマンド */
const uint8_t CMD_BOOT_LOADER_MODE[]	={0x7F};

const uint8_t BTL_CMD_GET[]			=	{0x00,0xff};	
const uint8_t BTL_CMD_GID[]			=	{0x02,0xfd};	
const uint8_t BTL_CMD_ReadMemory[]	=	{0x11,0xee};
const uint8_t BTL_CMD_WriteMemory[] =	{0x31,0xce};

#define ACK									0x79
#define NACK								0x1F



extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim14;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart4_tx;
extern DMA_HandleTypeDef hdma_usart4_rx;
extern uint8_t pc_rx_buff[10];
extern uint8_t pc_tx_buff[256];

typedef struct{
	uint8_t tx_buff[258];
	uint8_t rx_buff[258];
	void (*funcCallBack)(void);
	bool IsConnect;
	uint16_t rx_size;
	uint32_t firm_write_size;
	uint32_t latest_write_size;
}STM32_BTL_COM;

STM32_BTL_COM	stm32_btl_com = {0};


static inline void Wait_msec(uint32_t millisecond)
{
	__HAL_TIM_SET_COUNTER(&htim3,0);
	HAL_TIM_Base_Start(&htim3);
	while(__HAL_TIM_GET_COUNTER(&htim3) < millisecond) {
		asm("nop");
	}
	 HAL_TIM_Base_Stop(&htim3);
}

static  void PC_CommandParser(void);
static  void SetBootMode(bool enable);
static  void ResetTarget(bool enable);
static void Stm32BTL_SendRecvCommand(uint8_t *tx_buff,uint16_t tx_byte,uint8_t *rx_buff,uint16_t rx_byte,void (*call_back_func)(void));
static void Stm32BTL_RecvCommand(uint8_t *rx_buff,uint16_t rx_byte,void (*call_back_func)(void));
static void Stm32BTL_Connent(void);
static void Stm32BTL_CallBack_Connent(void);

static void Stm32BTL_CMD_GET(void);
static void Stm32BTL_CMD_CallBack1_GET(void);
static void Stm32BTL_CMD_CallBack2_GET(void);
static void Stm32BTL_CMD_CallBack3_GET(void);

static void Stm32BTL_CMD_FIRM_WRITE(void);
static void Stm32BTL_CMD_CallBack1_FW_WriteMemory(void);
static void Stm32BTL_CMD_CallBack2_FW_WriteMemory(void);
static void Stm32BTL_CMD_CallBack3_FW_WriteMemory(void);


static  void PC_CommandParser(void)
{
	int len;
	if(pc_rx_buff[0] == 'a') {

		len = sprintf((char*)pc_tx_buff,"Hello\r\n");
		HAL_UART_Transmit_DMA(&huart2, pc_tx_buff, len);
	}
	else if(pc_rx_buff[0] == 'b') {	/* 通常リセット */
		ResetTarget(true);
		SetBootMode(false);
		Wait_msec(100);
		ResetTarget(false);
		len = sprintf((char*)pc_tx_buff,"Normal Reset\r\n");
		HAL_UART_Transmit_DMA(&huart2, pc_tx_buff, len);
	}
	else if(pc_rx_buff[0] == 'c') {	/* boot mode */
		ResetTarget(true);
		SetBootMode(true);
		Wait_msec(10);
		ResetTarget(false);
		Wait_msec(5);
		Stm32BTL_Connent();
	}
	else if(pc_rx_buff[0] == 'd') {	/* Get CMD */
		Stm32BTL_CMD_GET();
	}
	else if(pc_rx_buff[0] == 'e') {	/* Get CMD */
		Stm32BTL_CMD_FIRM_WRITE();
	}
	HAL_UART_Receive_DMA(&huart2, pc_rx_buff, 1);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hUart)
{
	if(hUart->Instance == USART2) {
		PC_CommandParser();
	}
	else if(hUart->Instance == TARGET_USART) {
		if(stm32_btl_com.funcCallBack != NULL) {
			stm32_btl_com.funcCallBack();	
		}
	}
}

static  void SetBootMode(bool enable)
{
	static bool bFirst = true;
	if(enable) {
		HAL_GPIO_WritePin(GPIO_BOOT0_GPIO_Port,GPIO_BOOT0_Pin,1);
	}
	else {
		HAL_GPIO_WritePin(GPIO_BOOT0_GPIO_Port,GPIO_BOOT0_Pin,0);
	}

	if(bFirst) {
		bFirst = false;

	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /*Configure GPIO pin : GPIO_BOOT0_Pin */
	  GPIO_InitStruct.Pin = GPIO_BOOT0_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIO_BOOT0_GPIO_Port, &GPIO_InitStruct);
	}
}

static  void ResetTarget(bool enable)
{
	static bool bFirst = true;
	if(enable) {
		HAL_GPIO_WritePin(GPIO_NRST_GPIO_Port,GPIO_NRST_Pin,0);
	}
	else {
		HAL_GPIO_WritePin(GPIO_NRST_GPIO_Port,GPIO_NRST_Pin,1);
	}

	if(bFirst) {
		bFirst = false;

	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /*Configure GPIO pin : GPIO_NRST_Pin */
	  GPIO_InitStruct.Pin = GPIO_NRST_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIO_NRST_GPIO_Port, &GPIO_InitStruct);
	}
}

static void Stm32BTL_SendRecvCommand(uint8_t *tx_buff,uint16_t tx_byte,uint8_t *rx_buff,uint16_t rx_byte,void (*call_back_func)(void))
{
	stm32_btl_com.funcCallBack = call_back_func;
	HAL_UART_Transmit_DMA(&hTargetUart, tx_buff, tx_byte);
	stm32_btl_com.rx_size = rx_byte;
	HAL_UART_Receive_IT(&hTargetUart, rx_buff, rx_byte);	/* DMAに変更予定 */
}

static void Stm32BTL_RecvCommand(uint8_t *rx_buff,uint16_t rx_byte,void (*call_back_func)(void))
{
	stm32_btl_com.funcCallBack = call_back_func;
	stm32_btl_com.rx_size = rx_byte;
	HAL_UART_Receive_IT(&hTargetUart, rx_buff, rx_byte);	/* DMAに変更予定 */
}

static void Stm32BTL_Connent(void)
{
	/* タイムアウトエラーの処理が必要 */
	Stm32BTL_SendRecvCommand((uint8_t*)CMD_BOOT_LOADER_MODE,1,stm32_btl_com.rx_buff,1,Stm32BTL_CallBack_Connent);
}
static void Stm32BTL_CallBack_Connent(void)
{
	int32_t len;
	if(stm32_btl_com.rx_buff[0] == ACK) {
		len = sprintf((char*)pc_tx_buff,"ACK\r\n");
		HAL_UART_Transmit_DMA(&huart2, pc_tx_buff, len);
	}
	else if(stm32_btl_com.rx_buff[0] == NACK) {
		len = sprintf((char*)pc_tx_buff,"NACK\r\n");
		HAL_UART_Transmit_DMA(&huart2, pc_tx_buff, len);
	}
	else {
		len = sprintf((char*)pc_tx_buff,"unknown\r\n");
		HAL_UART_Transmit_DMA(&huart2, pc_tx_buff, len);
	}
}

static void Stm32BTL_CMD_GET(void)
{
	Stm32BTL_SendRecvCommand((uint8_t*)BTL_CMD_GET,2,stm32_btl_com.rx_buff,1,Stm32BTL_CMD_CallBack1_GET);
}

static void Stm32BTL_CMD_CallBack1_GET(void)
{
	int32_t len;
	if(stm32_btl_com.rx_buff[0] == ACK) {
		Stm32BTL_RecvCommand(stm32_btl_com.rx_buff,1,Stm32BTL_CMD_CallBack2_GET);
	}
	else {
		len = sprintf((char*)pc_tx_buff,"NACK\r\n");
		HAL_UART_Transmit_DMA(&huart2, pc_tx_buff, len);
	}
}

static void Stm32BTL_CMD_CallBack2_GET(void)
{
	int32_t len;
	uint16_t size = (uint16_t)stm32_btl_com.rx_buff[0] + 2;	/* ACK or NACK分加算 */

	len = sprintf((char*)pc_tx_buff, "GET Size:%u\r\n",size);
	HAL_UART_Transmit_DMA(&huart2, pc_tx_buff, len);

	Stm32BTL_RecvCommand(stm32_btl_com.rx_buff,size,Stm32BTL_CMD_CallBack3_GET);
}

static void Stm32BTL_CMD_CallBack3_GET(void)
{
	uint16_t i;
	uint32_t len = 0;

	for(i = 0; i < stm32_btl_com.rx_size -1;i++) {
		len += sprintf((char*)pc_tx_buff + len,"%02x ",stm32_btl_com.rx_buff[i]);
	}

	if(stm32_btl_com.rx_buff[i] == ACK) {
		len += sprintf((char*)pc_tx_buff + len,"\r\nGET ACK\r\n");
	}
	else if(stm32_btl_com.rx_buff[i] == NACK) {
		len += sprintf((char*)pc_tx_buff + len,"\r\nGET NACK\r\n");
	}
	else {
		len += sprintf((char*)pc_tx_buff + len,"\r\nGET unknown\r\n");
	}
	HAL_UART_Transmit_DMA(&huart2, pc_tx_buff, len);
}

static void Stm32BTL_CMD_FIRM_WRITE(void)
{
	stm32_btl_com.firm_write_size = 0;

	if(((G071_FIRM_SIZE % 4) != 0) || G071_FIRM_SIZE <= 0) {
		int32_t len;
		len = sprintf((char*)pc_tx_buff,"WriteMemory FirmSizeErr\r\n");
		HAL_UART_Transmit_DMA(&huart2, pc_tx_buff, len);
	}
	else {
		/* MemWriteコマンド送信 */
		Stm32BTL_SendRecvCommand((uint8_t*)BTL_CMD_WriteMemory,2,stm32_btl_com.rx_buff,1,Stm32BTL_CMD_CallBack1_FW_WriteMemory);
	}
}

static void Stm32BTL_CMD_CallBack1_FW_WriteMemory(void)
{
	if(stm32_btl_com.rx_buff[0] == ACK) {
		uint32_t start = G071_FIRM_START_ADDRESS + stm32_btl_com.firm_write_size;
		stm32_btl_com.tx_buff[0] = ((start >> 24) & 0x000000ff);
		stm32_btl_com.tx_buff[1] = ((start >> 16) & 0x000000ff);
		stm32_btl_com.tx_buff[2] = ((start >>  8) & 0x000000ff);
		stm32_btl_com.tx_buff[3] = ((start      ) & 0x000000ff);
		stm32_btl_com.tx_buff[4] = stm32_btl_com.tx_buff[0] ^ stm32_btl_com.tx_buff[1] ^ stm32_btl_com.tx_buff[2] ^ stm32_btl_com.tx_buff[3];
	
		stm32_btl_com.latest_write_size = 0;
		Stm32BTL_SendRecvCommand(stm32_btl_com.tx_buff,5,stm32_btl_com.rx_buff,1,Stm32BTL_CMD_CallBack2_FW_WriteMemory);
	}
	else {
		int32_t len;
		len = sprintf((char*)pc_tx_buff,"WriteMemory NACK\r\n");
		HAL_UART_Transmit_DMA(&huart2, pc_tx_buff, len);
	}
}

static void Stm32BTL_CMD_CallBack2_FW_WriteMemory(void)
{
		int32_t len;
	if(stm32_btl_com.rx_buff[0] == ACK) { /* StartAddress CheckSum OK*/
		uint8_t xor = 0;
		uint8_t data;
		uint16_t i;
		uint32_t size = G071_FIRM_SIZE - stm32_btl_com.firm_write_size;
		if(size == 0) {
			/* こないはず */
			len = sprintf((char*)pc_tx_buff,"WriteMemory Send Size Zero\r\n");
			HAL_UART_Transmit_DMA(&huart2, pc_tx_buff, len);
			return;
		}
		else if(size > 256) {
			size = 256;
		}
		stm32_btl_com.tx_buff[0] = (uint8_t)(size - 1);
		xor = stm32_btl_com.tx_buff[0];
		for(i = 0; i < size; i++) {
			data = G071_FirmBinaly[stm32_btl_com.firm_write_size + i];
			stm32_btl_com.tx_buff[i + 1] = data;
			xor ^= data;
		}
		stm32_btl_com.tx_buff[i + 1] = xor;

		stm32_btl_com.latest_write_size = size;

		Stm32BTL_SendRecvCommand(stm32_btl_com.tx_buff,size+2,stm32_btl_com.rx_buff,1,Stm32BTL_CMD_CallBack3_FW_WriteMemory);
	}
	else {
		len = sprintf((char*)pc_tx_buff,"WriteMemory Address Checksum NACK\r\n");
		HAL_UART_Transmit_DMA(&huart2, pc_tx_buff, len);
	}
}

static void Stm32BTL_CMD_CallBack3_FW_WriteMemory(void)
{
	int32_t len;
	if(stm32_btl_com.rx_buff[0] == ACK) { /* StartAddress CheckSum OK*/
		stm32_btl_com.firm_write_size += stm32_btl_com.latest_write_size;
		if(G071_FIRM_SIZE <= stm32_btl_com.firm_write_size) {
			len = sprintf((char*)pc_tx_buff,"WriteMemory WriteComplete NoCheck\r\n");
			HAL_UART_Transmit_DMA(&huart2, pc_tx_buff, len);
		}
		else {
			/* 次の書き込みへ */
			Stm32BTL_SendRecvCommand((uint8_t*)BTL_CMD_WriteMemory,2,stm32_btl_com.rx_buff,1,Stm32BTL_CMD_CallBack1_FW_WriteMemory);
		}
	}
	else {
		len = sprintf((char*)pc_tx_buff,"WriteMemory Address Checksum NACK\r\n");
		HAL_UART_Transmit_DMA(&huart2, pc_tx_buff, len);
	}
}

