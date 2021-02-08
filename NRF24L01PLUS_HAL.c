/*
 * NRF24L01PLUS_HAL.c
 *
 *  Created on: 23 СЏРЅРІ. 2021 Рі.
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

//Время запуска таймера для таймаута
uint32_t nrf_hal_timeout_start_tick[NRF_TIMERS_COUNT] = {0x00};

//Значение таймаута
uint32_t nrf_hal_timeout_value[NRF_TIMERS_COUNT] = {0x00};

//Инициализация HAL
uint8_t NRF_HAL_INIT(void)
{
	//Ничего не делаем, т.к. инициализация GPIO, SPI и таймера в другом месте

	//Только пауза в 100 мс для включения NRF24L01PLUS
	HAL_Delay(100);

	//Сброс линии CE в ноль
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

	return NRF_ERROR_CODE_SUCCESS;
}

//Обмен данными по SPI с приёмопередатчиком NRF24L01PLUS
uint8_t NRF_HAL_SPI_EXCHANGE(uint8_t *data_out, uint8_t *data_in, uint16_t size_data)
{
	uint8_t pause;

	//Проверка аргументов
	if((data_out != NULL) && (data_in != NULL) && (size_data != 0x00) && (size_data <= NRF_SPI_MAXIMUM_DATA_EXCHANGE_SIZE))
	{
		//Линию CSN в низкий уровень
		HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);

		//Пауза
		pause = 0x0A;
		while(pause--);

		//Обмен данными черезSPI
		HAL_SPI_EXCHANGE(data_out, data_in, size_data);

		//Пауза
		pause = 0x0A;
		while(pause--);

		//Линию CSN в высокий уровень
		HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);

		//Пауза
		pause = 0x0B;
		while(pause--);
	}
	else
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}

	return NRF_ERROR_CODE_SUCCESS;
}

//Пауза в микросеундах
uint8_t NRF_HAL_MICRO_SECOND_DELAY(uint32_t delay)
{
	HAL_TIM_MICROSECOND_DELAY(delay);

	return NRF_ERROR_CODE_SUCCESS;
}

//Запуск таймаута
uint8_t NRF_HAL_START_TIMEOUT(uint8_t number, uint32_t time)
{
	//Проверка аргумента
	if((time != 0x00) && (time < 10000) && (number < NRF_TIMERS_COUNT))
	{
		//Запомним текущее время и значение таймаута
		nrf_hal_timeout_value[number] = time;
		nrf_hal_timeout_start_tick[number] = HAL_GetTick();
	}
	else
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}

	return NRF_ERROR_CODE_SUCCESS;
}

//Остановка таймаута
uint8_t NRF_HAL_STOP_TIMEOUT(uint8_t number)
{
	//Сброс значения таймаута
	nrf_hal_timeout_value[number] = 0x00;

	return NRF_ERROR_CODE_SUCCESS;
}

//Статус таймаута
uint8_t NRF_HAL_STATUS_TIMEOUT(uint8_t number)
{
	uint32_t current_tick;

	//Получим текущее время
	current_tick = HAL_GetTick();

	//Проверка таймаута
	if((current_tick - nrf_hal_timeout_start_tick[number]) > nrf_hal_timeout_value[number])
	{
		return NRF_ERROR_CODE_TIMEOUT;
	}

	return NRF_ERROR_CODE_SUCCESS;
}

//Установить состояние вывода CE
uint8_t NRF_HAL_SET_CE_PIN_STATE(uint8_t state)
{
	//Меняем состояние вывода
	if(state == NRF_CE_STATE_ON)
	{
		//Линию CE в еденицу
		HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
	}
	else
	{
		//Линию CE в ноль
		HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
	}

	return NRF_ERROR_CODE_SUCCESS;
}

//Запросить состояние прерывания IRQ
uint8_t NRF_HAL_GET_IRQ_STATE(uint8_t *state)
{
	//Проверка аргумента
	if(state != NULL)
	{
		//Выдадим состояние прерывания
		if(HAL_GET_NRF_IRQ_STATUS() != COUNTER_STATE_OFF)
		{
			*state = NRF_STATE_ON;
			HAL_RST_NRF_IRQ_STATUS();//Сброс статуса прерывания
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
