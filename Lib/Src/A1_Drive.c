//
// Created by Trisoil on 2022/5/3.
//

#include "A1_Drive.h"
#include "dma.h"
#include "usart.h"

#define PI 3.1415926535

extern DMA_HandleTypeDef hdma_usart6_tx;

union Motor_Tx{
		uint8_t data[24];
		Send_Data Tx_Message;
}Motor_Tx_u;

union CRCC{
		uint8_t data[4];
		uint32_t crc;
}CRC_u;

uint8_t Data_Box[3][34];

// Message Sent Part

void Usart6_TX_DMA_Init(void)
{
	//enable the DMA transfer for the receiver request
	//使能DMA串口接收
	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);
}

void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{

	//disable DMA
	//失效DMA
	__HAL_DMA_DISABLE(&hdma_usart6_tx);

	while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart6_tx);
	}
	hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);
	//clear flag
	//清除标志位
	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF7);
	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_HTIF7);

	//set data address
	//设置数据地址
	hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
	//set data length
	//设置数据长度
	hdma_usart6_tx.Instance->NDTR = len;

	//
	//enable DMA
	//使能DMA
	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

	__HAL_DMA_ENABLE(&hdma_usart6_tx);

	while (!(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)){
		__HAL_DMA_ENABLE(&hdma_usart6_tx);
	}

}


uint32_t crc32_core(uint32_t *ptr, uint32_t len) {
	uint32_t xbit = 0;
	uint32_t data = 0;
	uint32_t CRC32 = 0xFFFFFFFF;
	const uint32_t dwPolynomial = 0x04c11db7;
	for (uint32_t i = 0; i < len; i++) {
		xbit = 1 << 31;
		data = ptr[i];
		for (uint32_t bits = 0; bits < 32; bits++) {
			if (CRC32 & 0x80000000) {
				CRC32 <<= 1;
				CRC32 ^= dwPolynomial;
			} else
				CRC32 <<= 1;
			if (data & xbit)
				CRC32 ^= dwPolynomial;
			xbit >>= 1;
		}
	}
	return CRC32;
}

void Control_Message_Send(int ID)
{
	uint32_t crc = crc32_core((uint32_t *) Data_Box[ID], 7);
	CRC_u.crc = crc;
	for (int i=0;i<24;i++) Data_Box[ID][i] = Motor_Tx_u.data[i];
	int cnt = 0;
	for (int i=30;i<34;i++){
		Data_Box[ID][i] = CRC_u.data[cnt++];
	}
	usart6_tx_dma_enable(Data_Box[ID], 34);
}

void Mode_Control(int ID,int Mode)
{
	Motor_Tx_u.Tx_Message.mode = Mode;
	Motor_Tx_u.Tx_Message.Motor_ID = ID;
	Motor_Tx_u.Tx_Message.start[0] = 0xFE;
	Motor_Tx_u.Tx_Message.start[1] = 0xEE;
}

void A1_Motor_Multiple_Control(int ID,int mode,float Torque,float W,float Position)
{
	if (mode == 1 || mode == 5)
	{
		Mode_Control(ID,mode);
		Control_Message_Send(ID);
		return;
	}
	Mode_Control(ID,10);
	// τ = τf f + kp · (pdes − p) + kd · (ωdes − ω)
	float kp = 0.2f;
	float kd = 0.5f;

	Motor_Tx_u.Tx_Message.T = (uint16_t)(Torque * 256.0f);
	Motor_Tx_u.Tx_Message.W = (uint16_t)(W * 128.0f * 9.1f);
	Motor_Tx_u.Tx_Message.Pos = (uint32_t)(Position * (16384.0f / (2.0f * PI)) * 9.1f);
//	Motor_Tx_u.Tx_Message.kp = (uint16_t)(0.2f*2048.0f);
//	Motor_Tx_u.Tx_Message.kw = (uint16_t)(3.0f*1024.0f);
	Control_Message_Send(ID);
}

void A1_Motor_Speed_Control(int ID,float W)
{
	Mode_Control(ID,10);
	Motor_Tx_u.Tx_Message.T = 0;
	Motor_Tx_u.Tx_Message.W = 	(uint16_t)(W * 128.0f * 9.1f);
	Motor_Tx_u.Tx_Message.Pos = 0;
	Motor_Tx_u.Tx_Message.kw = (uint16_t)(3.0f * 1024.0f);
	Control_Message_Send(ID);
}

void A1_Motor_Position_Control(int ID,float Position)
{
	Mode_Control(ID,10);
	Motor_Tx_u.Tx_Message.T = 0;
	Motor_Tx_u.Tx_Message.W = 0;
	Motor_Tx_u.Tx_Message.Pos = (uint32_t)(Position * (16384.0f / (2.0f * PI)) * 9.1f);
	Motor_Tx_u.Tx_Message.kp = (uint16_t)(0.2f * 2048.0f);
	Motor_Tx_u.Tx_Message.kw = (uint16_t)(3.0f * 1024.0f);
	Control_Message_Send(ID);
}

// Message Receive part

// 设置数据接收长度
#define SBUS_RX_BUF_NUM 78u

uint8_t A1_Motor_Rx_Data[2][SBUS_RX_BUF_NUM];

union Motor_Rx{
		uint8_t data[12];
		Rx_Data Received_data;
}Motor_Rx_u;

extern  DMA_HandleTypeDef hdma_usart6_rx;

void USART6_Receive_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num) {
	//enable the DMA transfer for the receiver request
	//使能DMA串口接收
	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);

	//enalbe idle interrupt
	//使能空闲中断
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	//HAL_UART_Receive_IT()
	//disable DMA
	//失效DMA
	__HAL_DMA_DISABLE(&hdma_usart6_rx);
	while (hdma_usart6_rx.Instance->CR & DMA_SxCR_EN) {
		__HAL_DMA_DISABLE(&hdma_usart6_rx);
	}

	hdma_usart6_rx.Instance->PAR = (uint32_t) &(USART6->DR);
	//memory buffer 1
	//内存缓冲区1
	hdma_usart6_rx.Instance->M0AR = (uint32_t) (rx1_buf);
	//memory buffer 2
	//内存缓冲区2
	hdma_usart6_rx.Instance->M1AR = (uint32_t) (rx2_buf);
	//data length
	//数据长度
	hdma_usart6_rx.Instance->NDTR = dma_buf_num;
	//enable double memory buffer
	//使能双缓冲区
	SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

	//enable DMA
	//使能DMA
	__HAL_DMA_ENABLE(&hdma_usart6_rx);

}

void USART6_Rx_Init() {
	USART6_Receive_init(A1_Motor_Rx_Data[0], A1_Motor_Rx_Data[1], SBUS_RX_BUF_NUM);
}

void Received_Data_Dealer(const uint8_t *sbus_buf)
{
	int ID = sbus_buf[2];
	Motor_Rx_u.data[0] = sbus_buf[4];
	Motor_Rx_u.data[1] = sbus_buf[6];
	Motor_Rx_u.data[2] = sbus_buf[12];
	Motor_Rx_u.data[3] = sbus_buf[13];
	Motor_Rx_u.data[4] = sbus_buf[14];
	Motor_Rx_u.data[5] = sbus_buf[15];
	Motor_Rx_u.data[6] = sbus_buf[26];
	Motor_Rx_u.data[7] = sbus_buf[27];
	Motor_Rx_u.data[8] = sbus_buf[30];
	Motor_Rx_u.data[9] = sbus_buf[31];
	Motor_Rx_u.data[10] = sbus_buf[32];
	Motor_Rx_u.data[11] = sbus_buf[33];
	A1_State.Mode = Motor_Rx_u.Received_data.mode;
	A1_State.Temp = Motor_Rx_u.Received_data.Temp;
	A1_State.Torque = ((float)Motor_Rx_u.Received_data.T)/256.0f;
	A1_State.Omega = ((float)Motor_Rx_u.Received_data.W)/128.0f;
	A1_State.Acc = Motor_Rx_u.Received_data.Acc;
	A1_State.Position = Motor_Rx_u.Received_data.Pos;
}

void USART6_IRQHandler(void) {
	if (huart6.Instance->SR & UART_FLAG_RXNE)//接收到数据
	{
		__HAL_UART_CLEAR_PEFLAG(&huart6);

	} else if (USART6->SR & UART_FLAG_IDLE) {
		static uint16_t this_time_rx_len = 0;

		__HAL_UART_CLEAR_PEFLAG(&huart6);

		if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
			/* Current memory buffer used is Memory 0 */

			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart6_rx);

			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

			//reset set_data_lenght
			//重新设定数据长度
			hdma_usart6_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

			//set memory buffer 1
			//设定缓冲区1
			hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart6_rx);

			if (this_time_rx_len == SBUS_RX_BUF_NUM){
				Received_Data_Dealer(A1_Motor_Rx_Data[0]);
			}
		} else {
			/* Current memory buffer used is Memory 1 */
			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart6_rx);

			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

			//reset set_data_length
			//重新设定数据长度
			hdma_usart6_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

			//set memory buffer 0
			//设定缓冲区0
			DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart6_rx);

			if (this_time_rx_len == SBUS_RX_BUF_NUM){
				Received_Data_Dealer(A1_Motor_Rx_Data[1]);
			}
		}
	}
}
