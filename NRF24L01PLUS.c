/*
 * NRF24L01PLUS.c
 *
 *  Created on: 23 янв. 2021 г.
 *      Author: K.Kuznetzov
 */

/* Includes ------------------------------------------------------------------*/
#include "NRF24L01PLUS_DEFINES.h"
#include "NRF24L01PLUS_HAL.h"
#include "NRF24L01PLUS.h"
#include <stdint.h>
#include <stdlib.h>

//Регистр статуса
uint8_t nrf_status_register = 0x00;

//Данные для записи и чтения
uint8_t nrf_data_out[NRF_SPI_MAXIMUM_DATA_EXCHANGE_SIZE];//Для записи
uint8_t nrf_data_in[NRF_SPI_MAXIMUM_DATA_EXCHANGE_SIZE];//Для чтения

//Состояние автомата обмена через радиоканал
uint8_t nrf_machine_state = NRF_MACHINE_STATE_FREE;

//Для хранения данных для отправки и приёма
uint8_t nrf_transmit_data[NRF_RF_MAXIMUM_DATA_EXCHANGE_SIZE];
uint8_t nrf_transmit_data_size;
uint8_t nrf_receive_data[NRF_RF_MAXIMUM_DATA_EXCHANGE_SIZE];
uint8_t nrf_receive_data_size;

//Длина адреса
uint8_t nrf_size_address = NRF_ADDRESS_LENGTH_5_BYTE_VALUE;

//Мощность и скорость обмена
uint8_t nrf_power_and_baudrate;

//Флаг отправки данных без подтверждения
uint8_t nrf_tx_no_ack_flag = NRF_STATE_OFF;

//Для хранения настроек PIPE: разрешение PIPE, разрешение ответа ACK, разрешение динамической длины данных
uint8_t nrf_enaa = 0x00;
uint8_t nrf_rxaddr = 0x00;
uint8_t nrf_dynpd= 0x00;
uint8_t nrf_feature = 0x00;

//Регистр конфигурации
uint8_t nrf_config = 0x00;

//Инициализация NRF24L01PLUS
uint8_t NRF_INIT(uint8_t size_address)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t reg_value;

	//Проверка аргументов
	if((size_address != NRF_ADDRESS_LENGTH_3_BYTE_VALUE) && (size_address != NRF_ADDRESS_LENGTH_4_BYTE_VALUE) && (size_address != NRF_ADDRESS_LENGTH_5_BYTE_VALUE))
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Настройки CRC и прерываний. Размер CRC 1 байт, CRC разрешён, прерывания разрешены
		reg_value = NRF_REGISTER_CONFIG_EN_CRC;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_CONFIG, 0x01, &reg_value);

		//Запомним
		nrf_config = reg_value;

		//Пока запретим все PIPE
		reg_value = 0x00;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_EN_AA, 0x01, &reg_value);

		//Запомним
		nrf_enaa = reg_value;

		//Пока запретим все PIPE
		reg_value = 0x00;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_EN_RXADDR, 0x01, &reg_value);

		//Запомним
		nrf_rxaddr = reg_value;

		//Настроим длину адреса
		if(size_address == NRF_ADDRESS_LENGTH_3_BYTE_VALUE) reg_value = NRF_ADDRESS_LENGTH_3_BYTE;
		if(size_address == NRF_ADDRESS_LENGTH_4_BYTE_VALUE) reg_value = NRF_ADDRESS_LENGTH_4_BYTE;
		if(size_address == NRF_ADDRESS_LENGTH_5_BYTE_VALUE) reg_value = NRF_ADDRESS_LENGTH_5_BYTE;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_SETUP_AW, 0x01, &reg_value);

		//Запомним длину адреса
		nrf_size_address = size_address;

		//Настроим параметры повтора отправки сообщения, 2000 мкс, 3 отправки
		reg_value = (0x07 << NRF_ARD_BIT_LEFT_SHIFT) | 0x03;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_SETUP_RETR, 0x01, &reg_value);

		//Номер канала по умолчанию
		reg_value = NRF_RADIO_CHANNEL_DEFAULT;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_RF_CH, 0x01, &reg_value);

		//Настройм скорость обмена и мощность
		//Скорость 250К для большей чувствительности, мощность 0 дБм
		reg_value = NRF_REGISTER_RF_SETUP_RF_DR_LOW | (NRF_RF_PWR_0DBM << NRF_RF_PWR_BIT_LEFT_SHIFT);
		status = NRF_WRITE_REGISTER(NRF_REGISTER_RF_SETUP, 0x01, &reg_value);

		//Запомним скорость и мощность
		nrf_power_and_baudrate = reg_value;

		//Сброс флагов прерываний
		reg_value = NRF_REGISTER_STATUS_RX_DR_BIT | NRF_REGISTER_STATUS_TX_DS_BIT | NRF_REGISTER_STATUS_MAX_RT_BIT;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_STATUS, 0x01, &reg_value);

		//Отчистка FIFO приёмника
		status = NRF_FLUSH_RX();

		//Отчистка FIFO передатчика
		status = NRF_FLUSH_TX();

		//Пока запретим динамическую длину пакетов и подтверждения для всеx PIPE
		reg_value = 0x00;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_DYNPD, 0x01, &reg_value);

		//Запомним
		nrf_dynpd = reg_value;

		//Пока запретим подтверждение, динамическую длину данных и подтверждения
		reg_value = 0x00;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_FEATURE, 0x01, &reg_value);

		//Запомним
		nrf_feature = reg_value;

		//Сброс регистра статусов
		reg_value = NRF_REGISTER_STATUS_RX_DR_BIT | NRF_REGISTER_STATUS_TX_DS_BIT | NRF_REGISTER_STATUS_MAX_RT_BIT;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_STATUS, 0x01, &reg_value);

		//Сброс автомата состояний
		nrf_machine_state = NRF_MACHINE_STATE_FREE;

		//Сброс размеров данных
		nrf_transmit_data_size = 0x00;
		nrf_receive_data_size = 0x00;
	}

	return status;
}

//Чтение регистра
uint8_t NRF_READ_REGISTER(uint8_t address, uint8_t size, uint8_t *value)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t i;

	//Проверка аргументов
	if(value == NULL)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Формируем команду чтения из регистра
		nrf_data_out[0x00] = NRF_SPI_CMD_R_REGISTER | (address & NRF_REGISTER_ADDRESS_MASK);
		for(i = 0x00; i < size; i++)
		{
			nrf_data_out[i + 0x01] = 0x00;
		}

		//Читаем регистр
		status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, size + 0x01);

		//Проверка статуса
		if(status == NRF_ERROR_CODE_SUCCESS)
		{
			//Формируем ответ
			nrf_status_register = nrf_data_in[0x00];
			for(i = 0x00; i < size; i++)
			{
				value[i] = nrf_data_in[i + 0x01];
			}
		}
	}

	return status;
}

//Запись регистра
uint8_t NRF_WRITE_REGISTER(uint8_t address, uint8_t size, uint8_t *value)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t i;

	//Проверка аргументов
	if(value == NULL)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Формируем команду записи в регистр
		nrf_data_out[0x00] = NRF_SPI_CMD_W_REGISTER | (address & NRF_REGISTER_ADDRESS_MASK);
		for(i = 0x00; i < size; i++)
		{
			nrf_data_out[i + 0x01] = value[i];
		}

		//Пишем в регистр
		status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, size + 0x01);

		//Проверка статуса
		if(status == NRF_ERROR_CODE_SUCCESS)
		{
			//Формируем ответ
			nrf_status_register = nrf_data_in[0x00];
		}
	}

	return status;
}

//Чтение принятых данных
uint8_t NRF_READ_RX_PAYLOAD(uint8_t size, uint8_t *data)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t i;

	//Проверка аргументов
	if(data == NULL)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Формируем команду чтения данных
		nrf_data_out[0x00] = NRF_SPI_R_RX_PAYLOAD;
		for(i = 0x00; i < size; i++)
		{
			nrf_data_out[i + 0x01] = 0x00;
		}

		//Пишем команду
		status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, size + 0x01);

		//Проверка статуса
		if(status == NRF_ERROR_CODE_SUCCESS)
		{
			//Формируем ответ
			nrf_status_register = nrf_data_in[0x00];
			for(i = 0x00; i < size; i++)
			{
				data[i] = nrf_data_in[i + 0x01];
			}
		}
	}

	return status;
}

//Запись данных на передачу
uint8_t NRF_WRITE_TX_PAYLOAD(uint8_t size, uint8_t *data)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t i;

	//Проверка аргументов
	if(data == NULL)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Формируем команду записи данных
		nrf_data_out[0x00] = NRF_SPI_W_TX_PAYLOAD;
		for(i = 0x00; i < size; i++)
		{
			nrf_data_out[i + 0x01] = data[i];
		}

		//Пишем команду
		status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, size + 0x01);

		//Проверка статуса
		if(status == NRF_ERROR_CODE_SUCCESS)
		{
			//Формируем ответ
			nrf_status_register = nrf_data_in[0x00];
		}
	}

	return status;
}

//Отчистка TX FIFO
uint8_t NRF_FLUSH_TX(void)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;

	//Формируем команду
	nrf_data_out[0x00] = NRF_SPI_FLUSH_TX;

	//Пишем команду
	status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, 0x01);

	//Проверка статуса
	if(status == NRF_ERROR_CODE_SUCCESS)
	{
		//Формируем ответ
		nrf_status_register = nrf_data_in[0x00];
	}

	return status;
}

//Отчистка RX FIFO
uint8_t NRF_FLUSH_RX(void)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;

	//Формируем команду
	nrf_data_out[0x00] = NRF_SPI_FLUSH_RX;

	//Пишем команду
	status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, 0x01);

	//Проверка статуса
	if(status == NRF_ERROR_CODE_SUCCESS)
	{
		//Формируем ответ
		nrf_status_register = nrf_data_in[0x00];
	}

	return status;
}

//Повторное использование последних переданных данных
uint8_t NRF_REUSE_TX_PAYLOAD(void)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;

	//Формируем команду
	nrf_data_out[0x00] = NRF_SPI_REUSE_TX_PL;

	//Пишем команду
	status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, 0x01);

	//Проверка статуса
	if(status == NRF_ERROR_CODE_SUCCESS)
	{
		//Формируем ответ
		nrf_status_register = nrf_data_in[0x00];
	}

	return status;
}

//Чтение размера данных на верху FIFO
uint8_t NRF_READ_RX_PAYLOAD_WIDTH(uint8_t *width)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;

	//Проверка аргументов
	if(width == NULL)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Формируем команду чтения ширины данных
		nrf_data_out[0x00] = NRF_SPI_R_RX_PL_WID;
		nrf_data_out[0x01] = 0x00;

		//Пишем команду
		status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, 0x02);

		//Проверка статуса
		if(status == NRF_ERROR_CODE_SUCCESS)
		{
			//Формируем ответ
			nrf_status_register = nrf_data_in[0x00];
			*width = nrf_data_in[0x01];
		}
	}

	return status;
}

//Запись данных для ответа ACK
uint8_t NRF_WRITE_ACK_PAYLOAD(uint8_t pipe, uint8_t size, uint8_t *data)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t i;

	//Проверка аргументов
	if(data == NULL)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Формируем команду записи данных
		nrf_data_out[0x00] = NRF_SPI_W_ACK_PAYLOAD | (pipe & NRF_REGISTER_PIPE_MASK);
		for(i = 0x00; i < size; i++)
		{
			nrf_data_out[i + 0x01] = data[i];
		}

		//Пишем команду
		status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, size + 0x01);

		//Проверка статуса
		if(status == NRF_ERROR_CODE_SUCCESS)
		{
			//Формируем ответ
			nrf_status_register = nrf_data_in[0x00];
		}
	}

	return status;
}

//Запись данных на передачу без подтверждения
uint8_t NRF_WRITE_TX_PAYLOAD_NO_ACK(uint8_t size, uint8_t *data)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t i;

	//Проверка аргументов
	if(data == NULL)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Формируем команду записи данных
		nrf_data_out[0x00] = NRF_SPI_W_TX_PAYLOAD_NO_ACK;
		for(i = 0x00; i < size; i++)
		{
			nrf_data_out[i + 0x01] = data[i];
		}

		//Пишем команду
		status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, size + 0x01);

		//Проверка статуса
		if(status == NRF_ERROR_CODE_SUCCESS)
		{
			//Формируем ответ
			nrf_status_register = nrf_data_in[0x00];
		}
	}

	return status;
}

//Команда NOP
uint8_t NRF_NOP(void)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;

	//Формируем команду
	nrf_data_out[0x00] = NRF_SPI_NOP;

	//Пишем команду
	status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, 0x01);

	//Проверка статуса
	if(status == NRF_ERROR_CODE_SUCCESS)
	{
		//Формируем ответ
		nrf_status_register = nrf_data_in[0x00];
	}

	return status;
}

//Запись адреса на приём
uint8_t NRF_WRITE_RX_ADDRESS(uint8_t pipe, uint8_t *addr, uint8_t size)
{
	uint8_t status;
	uint8_t reg_address;

	//Проверка аргументов
	if((pipe > NRF_RX_DATA_PIPE_NUMBER_5) || (size != nrf_size_address))
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Зададим адрес регистра для адреса приёма в зависимости от номера PIPE
		if(pipe == NRF_RX_DATA_PIPE_NUMBER_0) reg_address = NRF_REGISTER_RX_ADDR_P0;
		if(pipe == NRF_RX_DATA_PIPE_NUMBER_1) reg_address = NRF_REGISTER_RX_ADDR_P1;
		if(pipe == NRF_RX_DATA_PIPE_NUMBER_2) reg_address = NRF_REGISTER_RX_ADDR_P2;
		if(pipe == NRF_RX_DATA_PIPE_NUMBER_3) reg_address = NRF_REGISTER_RX_ADDR_P3;
		if(pipe == NRF_RX_DATA_PIPE_NUMBER_4) reg_address = NRF_REGISTER_RX_ADDR_P4;
		if(pipe == NRF_RX_DATA_PIPE_NUMBER_5) reg_address = NRF_REGISTER_RX_ADDR_P5;

		status = NRF_WRITE_REGISTER(reg_address, size, addr);
	}

	return status;
}

//Запись адреса на передачу
uint8_t NRF_WRITE_TX_ADDRESS(uint8_t *addr, uint8_t size)
{
	uint8_t status;

	//Проверка аргументов
	if(size != nrf_size_address)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		status = NRF_WRITE_REGISTER(NRF_REGISTER_TX_ADDR, size, addr);
	}

	return status;
}

//Задание радиоканала, 0 - 125
uint8_t NRF_SET_CHANNEL(uint8_t channel)
{
	uint8_t status;

	//Проверка аргументов
	if(channel > NRF_RADIO_CHANNEL_NUMBER_MAXIMUM)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		status = NRF_WRITE_REGISTER(NRF_REGISTER_RF_CH, 0x01, &channel);
	}

	return status;
}

//Задание параметров повторной передачи пакетов
uint8_t NRF_SET_RETRANSMIT(uint8_t ard, uint8_t arc)
{
	uint8_t status;
	uint8_t reg_value;

	//Проверка аргументов
	if((ard > NRF_ARD_MAXIMUM) || (arc > NRF_ARC_MAXIMUM))
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		reg_value = (ard << NRF_ARD_BIT_LEFT_SHIFT) | arc;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_SETUP_RETR, 0x01, &reg_value);
	}

	return status;
}

//Задать размер CRC, 1/2
uint8_t NRF_SET_CRC_SIZE(uint8_t size)
{
	uint8_t status;
	uint8_t reg_value;

	//Проверка аргументов
	if((size != NRF_CRC_LENGTH_1_BYTE) && (size != NRF_CRC_LENGTH_2_BYTE))
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Сбросим бит
		nrf_config &= ~NRF_REGISTER_CONFIG_CRCO;

		//Выставим биты
		nrf_config |= NRF_REGISTER_CONFIG_EN_CRC;
		if(size == NRF_CRC_LENGTH_2_BYTE) nrf_config |= NRF_REGISTER_CONFIG_CRCO;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_CONFIG, 0x01, &nrf_config);
	}

	return status;
}

//Задать мощность передачтчика
uint8_t NRF_SET_POWER(uint8_t power)
{
	uint8_t status;

	//Проверка аргументов
	if(power > NRF_RF_PWR_0DBM)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Сброс бит задания мощность
		nrf_power_and_baudrate &= ~NRF_REGISTER_RF_SETUP_RF_PWR_MASK;

		//Зааём новое значение мощности
		nrf_power_and_baudrate = (power << NRF_RF_PWR_BIT_LEFT_SHIFT);
		status = NRF_WRITE_REGISTER(NRF_REGISTER_RF_SETUP, 0x01, &nrf_power_and_baudrate);
	}

	return status;
}

//Задать скорость обмена
uint8_t NRF_SET_BAUDRATE(uint8_t baudrate)
{
	uint8_t status;

	//Проверка аргументов
	if(baudrate > NRF_DATA_RATE_2M)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Новое значение скорости
		if(baudrate == NRF_DATA_RATE_250K)
		{
			nrf_power_and_baudrate &= ~NRF_REGISTER_RF_SETUP_RF_DR_HIGH;
			nrf_power_and_baudrate |= NRF_REGISTER_RF_SETUP_RF_DR_LOW;
		}
		if(baudrate == NRF_DATA_RATE_1M)
		{
			nrf_power_and_baudrate &= ~NRF_REGISTER_RF_SETUP_RF_DR_HIGH;
			nrf_power_and_baudrate &= ~NRF_REGISTER_RF_SETUP_RF_DR_LOW;
		}
		if(baudrate == NRF_DATA_RATE_2M)
		{
			nrf_power_and_baudrate |= NRF_REGISTER_RF_SETUP_RF_DR_HIGH;
			nrf_power_and_baudrate &= ~NRF_REGISTER_RF_SETUP_RF_DR_LOW;
		}

		status = NRF_WRITE_REGISTER(NRF_REGISTER_RF_SETUP, 0x01, &nrf_power_and_baudrate);
	}

	return status;
}

//Задать размер данных на приём для заданного PIPE, от 0 (не используется) до 32, для режима Static Length
uint8_t NRF_SET_RX_PAYLOAD_LENGTH(uint8_t pipe, uint8_t length)
{
	uint8_t status;
	uint8_t reg_address;

	//Проверка аргументов
	if((pipe > NRF_RX_DATA_PIPE_NUMBER_5) || (length > NRF_RF_MAXIMUM_DATA_EXCHANGE_SIZE))
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Зададим адрес регистра длины принимаемых данных в зависимости от номера PIPE
		if(pipe == NRF_RX_DATA_PIPE_NUMBER_0) reg_address = NRF_REGISTER_RX_PW_P0;
		if(pipe == NRF_RX_DATA_PIPE_NUMBER_1) reg_address = NRF_REGISTER_RX_PW_P1;
		if(pipe == NRF_RX_DATA_PIPE_NUMBER_2) reg_address = NRF_REGISTER_RX_PW_P2;
		if(pipe == NRF_RX_DATA_PIPE_NUMBER_3) reg_address = NRF_REGISTER_RX_PW_P3;
		if(pipe == NRF_RX_DATA_PIPE_NUMBER_4) reg_address = NRF_REGISTER_RX_PW_P4;
		if(pipe == NRF_RX_DATA_PIPE_NUMBER_5) reg_address = NRF_REGISTER_RX_PW_P5;

		status = NRF_WRITE_REGISTER(reg_address, 0x01, &length);
	}

	return status;
}

//Настройка PIPE, разрешить приём, разрешить ответ, разрешить динамическую длину ответа, данные в ответе
uint8_t NRF_SET_PIPE(uint8_t pipe, uint8_t en_pipe, uint8_t en_ack, uint8_t en_dpl, uint8_t en_ack_payload)
{
	uint8_t status;

	//Проверка аргументов
	if(pipe > NRF_RX_DATA_PIPE_NUMBER_5)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Разрешён PIPE
		if(en_pipe == NRF_STATE_ON)
		{
			//Если разрешён
			nrf_rxaddr |= (NRF_REGISTER_EN_RXADDR_ERX_0 << pipe);
		}
		else
		{
			//Если запрещён
			nrf_rxaddr &= ~(NRF_REGISTER_EN_RXADDR_ERX_0 << pipe);
		}

		//Пишем в регистр
		status = NRF_WRITE_REGISTER(NRF_REGISTER_EN_RXADDR, 0x01, &nrf_rxaddr);

		//Разрешён ли ACK для PIPE
		if(en_ack == NRF_STATE_ON)
		{
			//Если разрешён
			nrf_enaa |= (NRF_REGISTER_EN_AA_ENAA_P0 << pipe);
		}
		else
		{
			//Если запрещён
			nrf_enaa &= ~(NRF_REGISTER_EN_AA_ENAA_P0 << pipe);
		}

		//Пишем в регистр
		status = NRF_WRITE_REGISTER(NRF_REGISTER_EN_AA, 0x01, &nrf_enaa);

		//Разрешена ли динамическая длина данных
		if(en_dpl == NRF_STATE_ON)
		{
			//Если разрешён
			nrf_dynpd |= (NRF_REGISTER_DYNPD_DPL_P0 << pipe);
		}
		else
		{
			//Если запрещён
			nrf_dynpd &= ~(NRF_REGISTER_DYNPD_DPL_P0 << pipe);
		}

		//Разрешены ли данные в ответе ACK
		if(en_ack_payload == NRF_STATE_ON)
		{
			//Если разрешён
			//Тут же включим EN_DYN_ACK
			//Тут же нужно включить для приёма ACK бит DYN_PD0
			nrf_feature |= NRF_REGISTER_FEATURE_EN_ACK_PAY;
			nrf_feature |= NRF_REGISTER_FEATURE_EN_DYN_ACK;
			nrf_dynpd |= NRF_REGISTER_DYNPD_DPL_P0;
		}
		else
		{
			//Если запрещён
			//Тут же выключим EN_DYN_ACK
			nrf_feature &= ~NRF_REGISTER_FEATURE_EN_ACK_PAY;
			nrf_feature &= ~NRF_REGISTER_FEATURE_EN_DYN_ACK;
		}

		//Пишем в регистр
		status = NRF_WRITE_REGISTER(NRF_REGISTER_DYNPD, 0x01, &nrf_dynpd);

		//Если выставлен хоть один бит NRF_REGISTER_DYNPD, то нужно включить бит EN_DPL регистра NRF_REGISTER_FEATURE
		//Если ноль, то выключить
		if(nrf_dynpd != 0x00)
		{
			//Включим
			nrf_feature |= NRF_REGISTER_FEATURE_EN_DPL;
		}
		else
		{
			//Выключим
			nrf_feature &= ~NRF_REGISTER_FEATURE_EN_DPL;
		}

		//Пишем в регистр
		status = NRF_WRITE_REGISTER(NRF_REGISTER_FEATURE, 0x01, &nrf_feature);
	}

	return status;
}

//Запуск передачи пакета
uint8_t NRF_START_TRANSMIT(uint8_t *data, uint8_t size, uint32_t timeout, uint8_t no_ack_flag)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t reg_value;
	uint8_t i;

	//Проверка аргументов
	if((data == NULL) || (size == 0x00) || (timeout == 0x00))
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Проверка автомата состояний
		if(nrf_machine_state != NRF_MACHINE_STATE_FREE)
		{
			//Автомта состояний занят
			return NRF_ERROR_CODE_CHANNEL_BUSY;
		}
		else
		{
			//Линию CE в 0
			NRF_HAL_SET_CE_PIN_STATE(NRF_CE_STATE_OFF);

			//Выставим состояние автомата
			nrf_machine_state = NRF_MACHINE_STATE_START_SEND;

			//Запуск таймаута на ожидание завершения передачи
			NRF_HAL_START_TIMEOUT(NRF_TIMER_TIMEOUT, timeout);

			//Включаем приёмопередатчик, PWR_UP = 1
			nrf_config |= NRF_REGISTER_CONFIG_PWR_UP;
			status = NRF_WRITE_REGISTER(NRF_REGISTER_CONFIG, 0x01, &nrf_config);

			//Запуск таймаута для паузы на время включения
			NRF_HAL_START_TIMEOUT(NRF_TIMER_DELAY, NRF_TIME_POWER_UP);

			//Сохраним данные для передачи
			for(i = 0x00; i < size; i++)
			{
				nrf_transmit_data[i] = data[i];
			}
			nrf_transmit_data_size = size;

			//Флаг передачи без подтверждения
			if(no_ack_flag == NRF_STATE_ON)
			{
				nrf_tx_no_ack_flag = NRF_STATE_ON;
			}
			else
			{
				nrf_tx_no_ack_flag = NRF_STATE_OFF;
			}
		}
	}

	return status;
}

//Запуск приёма пакета
uint8_t NRF_START_RECEIVE(uint8_t *data, uint8_t size, uint32_t timeout)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t reg_value;
	uint8_t i;

	//Проверка аргументов
	if(timeout == 0x00)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Проверка автомата состояний
		if(nrf_machine_state != NRF_MACHINE_STATE_FREE)
		{
			//Автомта состояний занят
			return NRF_ERROR_CODE_CHANNEL_BUSY;
		}
		else
		{
			//Линию CE в 0
			NRF_HAL_SET_CE_PIN_STATE(NRF_CE_STATE_OFF);

			//Выставим состояние автомата
			nrf_machine_state = NRF_MACHINE_STATE_START_RECEIVE;

			//Запуск таймаута на ожидание завершения передачи
			NRF_HAL_START_TIMEOUT(NRF_TIMER_TIMEOUT, timeout);

			//Включаем приёмопередатчик, PWR_UP = 1
			nrf_config |= NRF_REGISTER_CONFIG_PWR_UP;
			status = NRF_WRITE_REGISTER(NRF_REGISTER_CONFIG, 0x01, &nrf_config);

			//Запуск таймаута для паузы на время включения
			NRF_HAL_START_TIMEOUT(NRF_TIMER_DELAY, NRF_TIME_POWER_UP);

			//Сохраним данные для передачи ACK
			for(i = 0x00; i < size; i++)
			{
				nrf_transmit_data[i] = data[i];
			}
			nrf_transmit_data_size = size;

			//Сброс размера данных
			nrf_receive_data_size = 0x00;
		}
	}

	return status;
}

//Получить принятые данные
uint8_t NRF_GET_RECEIVED_DATA(uint8_t *data, uint8_t buffer_size, uint8_t *data_size)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t i;

	//Проверка аргументов
	if((data == NULL) || (data_size == NULL) || (buffer_size < nrf_receive_data_size))
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//Выдадим данные
		for(i = 0x00; i < nrf_receive_data_size; i++)
		{
			data[i] = nrf_receive_data[i];
		}
		*data_size = nrf_receive_data_size;

		//Сброс размера данных
		nrf_receive_data_size = 0x00;
	}

	return status;
}

//Обработка приёма и передачи
uint8_t NRF_EXCHANGE_PROCESSING(void)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t reg_value;
	uint8_t irq_state = NRF_STATE_OFF;

	//Поведение зависит от состояния автомата

	//Была запущена передача данных
	if(nrf_machine_state == NRF_MACHINE_STATE_START_SEND)
	{
		//Запущена передача данных

		//Ожидаем истечения паузы включение приёмопередатчика
		if(NRF_HAL_STATUS_TIMEOUT(NRF_TIMER_DELAY) == NRF_ERROR_CODE_TIMEOUT)
		{
			//Пауза истекла, меняем состояние автомата
			nrf_machine_state = NRF_MACHINE_STATE_WAIT_SEND;

			//Загружаем данные в FIFO на передачу
			if(nrf_tx_no_ack_flag == NRF_STATE_ON)
			{
				//Если без подтверждения
				NRF_WRITE_TX_PAYLOAD_NO_ACK(nrf_transmit_data_size, nrf_transmit_data);
			}
			else
			{
				//Если с подтверждением
				NRF_WRITE_TX_PAYLOAD(nrf_transmit_data_size, nrf_transmit_data);
			}

			//Сброс размера данных
			nrf_transmit_data_size = 0x00;

			//Пауза
			NRF_HAL_MICRO_SECOND_DELAY(NRF_TIME_CE_START_TRANSMIT);

			//Выставляем сигнал CE в 1
			NRF_HAL_SET_CE_PIN_STATE(NRF_CE_STATE_ON);

			//Пауза
			NRF_HAL_MICRO_SECOND_DELAY(NRF_TIME_CE_START_TRANSMIT);

			//Выставляем сигнал CE в 0
			NRF_HAL_SET_CE_PIN_STATE(NRF_CE_STATE_OFF);

		    //Остановим таймер
			NRF_HAL_STOP_TIMEOUT(NRF_TIMER_DELAY);
		}
	}
	else
	//Ждём завершения отправки данных
	if(nrf_machine_state == NRF_MACHINE_STATE_WAIT_SEND)
	{
		//Проверяем статус прерывания
		//Если было прерывание, то читаем статус и из него смотри, что за прерывание
		//Либо ждём истечения таймаута

		//Запрос состояния прерывания
		NRF_HAL_GET_IRQ_STATE(&irq_state);

		//Смотрим было ли прерывание
		if(irq_state == NRF_STATE_ON)
		{
			//Было прерывание, нужно прочесть регистр статуса, достаточно команды NOP
			NRF_NOP();

			//Смотрим биты прерывания в регистре статуса

			//Бит прерывания завершения передачи данных
			if(nrf_status_register & NRF_REGISTER_STATUS_TX_DS_BIT)
			{
				//Успешно передали данные
				//И приняли ответ ACK с данными

				//Читаем длину данных в приёмном FIFO
				NRF_READ_RX_PAYLOAD_WIDTH(&nrf_receive_data_size);

				//Вычитываем данные
				NRF_READ_RX_PAYLOAD(nrf_receive_data_size, nrf_receive_data);

				//Остановка приёма или передачи данных
				status = NRF_STOP_PROCESSING();

				//Выставим статус завершения передачи
				status = NRF_ERROR_CODE_SEND_DATA_SUCCESS;
			}

			//Бит прерывания максимального числа попыток передачи данных
			if(nrf_status_register & NRF_REGISTER_STATUS_MAX_RT_BIT)
			{
				//Ошибка превышения максимального числа попыток отправки данных

				//Остановка приёма или передачи данных
				status = NRF_STOP_PROCESSING();

				//Выставим статус ошибки
				status = NRF_ERROR_CODE_SEND_MAX_RT_ERROR;
			}
		}
		else
		{
			//Не было прерывания, проверим таймаут
			if(NRF_HAL_STATUS_TIMEOUT(NRF_TIMER_TIMEOUT) == NRF_ERROR_CODE_TIMEOUT)
			{
				//Таймаут истёк

				//Остановка приёма или передачи данных
				status = NRF_STOP_PROCESSING();

				//Выставим статус таймаута
				status = NRF_ERROR_CODE_TIMEOUT;
			}
		}
	}
	else
	//Был запущен приём данных
	if(nrf_machine_state == NRF_MACHINE_STATE_START_RECEIVE)
	{
		//Запущен приём данных

		//Ожидаем истечения паузы включение приёмопередатчика
		if(NRF_HAL_STATUS_TIMEOUT(NRF_TIMER_DELAY) == NRF_ERROR_CODE_TIMEOUT)
		{
			//Пауза истекла, меняем состояние автомата
			nrf_machine_state = NRF_MACHINE_STATE_WAIT_RECEIVE;

			//Загружаем данные в FIFO на передачу ACK, PIPE 1 для приёма данных
			NRF_WRITE_ACK_PAYLOAD(NRF_RX_DATA_PIPE_NUMBER_1, nrf_transmit_data_size, nrf_transmit_data);

			//Выставим бит PRIM_RX в 1
			nrf_config |= NRF_REGISTER_CONFIG_PWR_UP | NRF_REGISTER_CONFIG_PRIM_RX;
			status = NRF_WRITE_REGISTER(NRF_REGISTER_CONFIG, 0x01, &nrf_config);

			//Пауза
			NRF_HAL_MICRO_SECOND_DELAY(NRF_TIME_CE_START_RECEIVE);

			//Выставляем сигнал CE в 1
			NRF_HAL_SET_CE_PIN_STATE(NRF_CE_STATE_ON);

		    //Остановим таймер
			NRF_HAL_STOP_TIMEOUT(NRF_TIMER_DELAY);
		}
	}
	else
	//Ждём завершения приёма данных
	if(nrf_machine_state == NRF_MACHINE_STATE_WAIT_RECEIVE)
	{
		//Проверяем статус прерывания
		//Если было прерывание, то читаем статус и из него смотри, что за прерывание
		//Либо ждём истечения таймаута

		//Запрос состояния прерывания
		NRF_HAL_GET_IRQ_STATE(&irq_state);

		//Смотрим было ли прерывание
		if(irq_state == NRF_STATE_ON)
		{
			//Было прерывание, нужно прочесть регистр статуса, достаточно команды NOP
			NRF_NOP();

			//Смотрим биты прерывания в регистре статуса

			//Бит прерывания приёма данных
			if(nrf_status_register & NRF_REGISTER_STATUS_RX_DR_BIT)
			{
				//Успешно приняли данные

				//Читаем длину данных в приёмном FIFO
				NRF_READ_RX_PAYLOAD_WIDTH(&nrf_receive_data_size);

				//Вычитываем данные
				NRF_READ_RX_PAYLOAD(nrf_receive_data_size, nrf_receive_data);

				//Остановка приёма или передачи данных
				status = NRF_STOP_PROCESSING();

				//Выставим статус завершения приёма
				status = NRF_ERROR_CODE_RECEIVE_DATA_SUCCESS;
			}
		}
		else
		{
			//Не было прерывания, проверим таймаут
			if(NRF_HAL_STATUS_TIMEOUT(NRF_TIMER_TIMEOUT) == NRF_ERROR_CODE_TIMEOUT)
			{
				//Таймаут истёк

				//Остановка приёма или передачи данных
				status = NRF_STOP_PROCESSING();

				//Выставим статус таймаута
				status = NRF_ERROR_CODE_TIMEOUT;
			}
		}
	}

	return status;
}

//Остановка приёма или передачи данных
uint8_t NRF_STOP_PROCESSING(void)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t reg_value;

	//Остановим таймер
	NRF_HAL_STOP_TIMEOUT(NRF_TIMER_TIMEOUT);

	//Выставляем сигнал CE в 0
	NRF_HAL_SET_CE_PIN_STATE(NRF_CE_STATE_OFF);

	//Сброс регистра статусов
	reg_value = NRF_REGISTER_STATUS_RX_DR_BIT | NRF_REGISTER_STATUS_TX_DS_BIT | NRF_REGISTER_STATUS_MAX_RT_BIT;
	status = NRF_WRITE_REGISTER(NRF_REGISTER_STATUS, 0x01, &reg_value);

	//Сброс FIFO на передачу и приём
	status = NRF_FLUSH_TX();
	status = NRF_FLUSH_RX();

	//Выключим приёмопередатчик, PWR_UP = 0, PRIM_RX = 0
	nrf_config &= ~NRF_REGISTER_CONFIG_PWR_UP;
	nrf_config &= ~NRF_REGISTER_CONFIG_PRIM_RX;
	status = NRF_WRITE_REGISTER(NRF_REGISTER_CONFIG, 0x01, &nrf_config);

	//Сброс автомата состояний
	nrf_machine_state = NRF_MACHINE_STATE_FREE;

	return status;
}
