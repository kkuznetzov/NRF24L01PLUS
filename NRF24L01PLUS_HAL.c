/*
 * NRF24L01PLUS_HAL.c
 *
 *  Created on: 23 янв. 2021 г.
 *      Author: K.Kuznetzov
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "iwdg.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "COUNTERHAL.h"
#include "NRF24L01PLUS_DEFINES.h"
#include "NRF24L01PLUS_HAL.h"

//����� ������� ������� ��� ��������
uint32_t nrf_hal_timeout_start_tick[NRF_TIMERS_COUNT] = {0x00};

//�������� ��������
uint32_t nrf_hal_timeout_value[NRF_TIMERS_COUNT] = {0x00};

//������������� HAL
uint8_t NRF_HAL_INIT(void)
{
	//������ �� ������, �.�. ������������� GPIO, SPI � ������� � ������ �����

	//������ ����� � 100 �� ��� ��������� NRF24L01PLUS
	HAL_Delay(100);

	//����� ����� CE � ����
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

	return NRF_ERROR_CODE_SUCCESS;
}

//����� ������� �� SPI � ����������������� NRF24L01PLUS
uint8_t NRF_HAL_SPI_EXCHANGE(uint8_t *data_out, uint8_t *data_in, uint16_t size_data)
{
	uint8_t pause;

	//�������� ����������
	if((data_out != NULL) && (data_in != NULL) && (size_data != 0x00) && (size_data <= NRF_SPI_MAXIMUM_DATA_EXCHANGE_SIZE))
	{
		//����� CSN � ������ �������
		HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);

		//�����
		pause = 0x0A;
		while(pause--);

		//����� ������� �����SPI
		HAL_SPI_EXCHANGE(data_out, data_in, size_data);

		//�����
		pause = 0x0A;
		while(pause--);

		//����� CSN � ������� �������
		HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);

		//�����
		pause = 0x0B;
		while(pause--);
	}
	else
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}

	return NRF_ERROR_CODE_SUCCESS;
}

//����� � ������������
uint8_t NRF_HAL_MICRO_SECOND_DELAY(uint32_t delay)
{
	HAL_TIM_MICROSECOND_DELAY(delay);

	return NRF_ERROR_CODE_SUCCESS;
}

//������ ��������
uint8_t NRF_HAL_START_TIMEOUT(uint8_t number, uint32_t time)
{
	//�������� ���������
	if((time != 0x00) && (time < 10000) && (number < NRF_TIMERS_COUNT))
	{
		//�������� ������� ����� � �������� ��������
		nrf_hal_timeout_value[number] = time;
		nrf_hal_timeout_start_tick[number] = HAL_GetTick();
	}
	else
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}

	return NRF_ERROR_CODE_SUCCESS;
}

//��������� ��������
uint8_t NRF_HAL_STOP_TIMEOUT(uint8_t number)
{
	//����� �������� ��������
	nrf_hal_timeout_value[number] = 0x00;

	return NRF_ERROR_CODE_SUCCESS;
}

//������ ��������
uint8_t NRF_HAL_STATUS_TIMEOUT(uint8_t number)
{
	uint32_t current_tick;

	//������� ������� �����
	current_tick = HAL_GetTick();

	//�������� ��������
	if((current_tick - nrf_hal_timeout_start_tick[number]) > nrf_hal_timeout_value[number])
	{
		return NRF_ERROR_CODE_TIMEOUT;
	}

	return NRF_ERROR_CODE_SUCCESS;
}

//���������� ��������� ������ CE
uint8_t NRF_HAL_SET_CE_PIN_STATE(uint8_t state)
{
	//������ ��������� ������
	if(state == NRF_CE_STATE_ON)
	{
		//����� CE � �������
		HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
	}
	else
	{
		//����� CE � ����
		HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
	}

	return NRF_ERROR_CODE_SUCCESS;
}

//��������� ��������� ���������� IRQ
uint8_t NRF_HAL_GET_IRQ_STATE(uint8_t *state)
{
	//�������� ���������
	if(state != NULL)
	{
		//������� ��������� ����������
		if(HAL_GET_NRF_IRQ_STATUS() != COUNTER_STATE_OFF)
		{
			*state = NRF_STATE_ON;
			HAL_RST_NRF_IRQ_STATUS();//����� ������� ����������
		}
		else
		{
			*state = NRF_STATE_OFF;
		}
	}
	else
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}

	return NRF_ERROR_CODE_SUCCESS;
}
