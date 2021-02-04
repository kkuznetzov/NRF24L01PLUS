/*
 * NRF24L01PLUS.c
 *
 *  Created on: 23 ���. 2021 �.
 *      Author: K.Kuznetzov
 */

/* Includes ------------------------------------------------------------------*/
#include "NRF24L01PLUS_DEFINES.h"
#include "NRF24L01PLUS_HAL.h"
#include "NRF24L01PLUS.h"
#include <stdint.h>
#include <stdlib.h>

//������� �������
uint8_t nrf_status_register = 0x00;

//������ ��� ������ � ������
uint8_t nrf_data_out[NRF_SPI_MAXIMUM_DATA_EXCHANGE_SIZE];//��� ������
uint8_t nrf_data_in[NRF_SPI_MAXIMUM_DATA_EXCHANGE_SIZE];//��� ������

//��������� �������� ������ ����� ����������
uint8_t nrf_machine_state = NRF_MACHINE_STATE_FREE;

//��� �������� ������ ��� �������� � �����
uint8_t nrf_transmit_data[NRF_RF_MAXIMUM_DATA_EXCHANGE_SIZE];
uint8_t nrf_transmit_data_size;
uint8_t nrf_receive_data[NRF_RF_MAXIMUM_DATA_EXCHANGE_SIZE];
uint8_t nrf_receive_data_size;

//����� ������
uint8_t nrf_size_address = NRF_ADDRESS_LENGTH_5_BYTE_VALUE;

//�������� � �������� ������
uint8_t nrf_power_and_baudrate;

//���� �������� ������ ��� �������������
uint8_t nrf_tx_no_ack_flag = NRF_STATE_OFF;

//��� �������� �������� PIPE: ���������� PIPE, ���������� ������ ACK, ���������� ������������ ����� ������
uint8_t nrf_enaa = 0x00;
uint8_t nrf_rxaddr = 0x00;
uint8_t nrf_dynpd= 0x00;
uint8_t nrf_feature = 0x00;

//������� ������������
uint8_t nrf_config = 0x00;

//������������� NRF24L01PLUS
uint8_t NRF_INIT(uint8_t size_address)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t reg_value;

	//�������� ����������
	if((size_address != NRF_ADDRESS_LENGTH_3_BYTE_VALUE) && (size_address != NRF_ADDRESS_LENGTH_4_BYTE_VALUE) && (size_address != NRF_ADDRESS_LENGTH_5_BYTE_VALUE))
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//��������� CRC � ����������. ������ CRC 1 ����, CRC ��������, ���������� ���������
		reg_value = NRF_REGISTER_CONFIG_EN_CRC;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_CONFIG, 0x01, &reg_value);

		//��������
		nrf_config = reg_value;

		//���� �������� ��� PIPE
		reg_value = 0x00;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_EN_AA, 0x01, &reg_value);

		//��������
		nrf_enaa = reg_value;

		//���� �������� ��� PIPE
		reg_value = 0x00;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_EN_RXADDR, 0x01, &reg_value);

		//��������
		nrf_rxaddr = reg_value;

		//�������� ����� ������
		if(size_address == NRF_ADDRESS_LENGTH_3_BYTE_VALUE) reg_value = NRF_ADDRESS_LENGTH_3_BYTE;
		if(size_address == NRF_ADDRESS_LENGTH_4_BYTE_VALUE) reg_value = NRF_ADDRESS_LENGTH_4_BYTE;
		if(size_address == NRF_ADDRESS_LENGTH_5_BYTE_VALUE) reg_value = NRF_ADDRESS_LENGTH_5_BYTE;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_SETUP_AW, 0x01, &reg_value);

		//�������� ����� ������
		nrf_size_address = size_address;

		//�������� ��������� ������� �������� ���������, 2000 ���, 3 ��������
		reg_value = (0x07 << NRF_ARD_BIT_LEFT_SHIFT) | 0x03;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_SETUP_RETR, 0x01, &reg_value);

		//����� ������ �� ���������
		reg_value = NRF_RADIO_CHANNEL_DEFAULT;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_RF_CH, 0x01, &reg_value);

		//�������� �������� ������ � ��������
		//�������� 250� ��� ������� ����������������, �������� 0 ���
		reg_value = NRF_REGISTER_RF_SETUP_RF_DR_LOW | (NRF_RF_PWR_0DBM << NRF_RF_PWR_BIT_LEFT_SHIFT);
		status = NRF_WRITE_REGISTER(NRF_REGISTER_RF_SETUP, 0x01, &reg_value);

		//�������� �������� � ��������
		nrf_power_and_baudrate = reg_value;

		//����� ������ ����������
		reg_value = NRF_REGISTER_STATUS_RX_DR_BIT | NRF_REGISTER_STATUS_TX_DS_BIT | NRF_REGISTER_STATUS_MAX_RT_BIT;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_STATUS, 0x01, &reg_value);

		//�������� FIFO ��������
		status = NRF_FLUSH_RX();

		//�������� FIFO �����������
		status = NRF_FLUSH_TX();

		//���� �������� ������������ ����� ������� � ������������� ��� ���x PIPE
		reg_value = 0x00;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_DYNPD, 0x01, &reg_value);

		//��������
		nrf_dynpd = reg_value;

		//���� �������� �������������, ������������ ����� ������ � �������������
		reg_value = 0x00;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_FEATURE, 0x01, &reg_value);

		//��������
		nrf_feature = reg_value;

		//����� �������� ��������
		reg_value = NRF_REGISTER_STATUS_RX_DR_BIT | NRF_REGISTER_STATUS_TX_DS_BIT | NRF_REGISTER_STATUS_MAX_RT_BIT;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_STATUS, 0x01, &reg_value);

		//����� �������� ���������
		nrf_machine_state = NRF_MACHINE_STATE_FREE;

		//����� �������� ������
		nrf_transmit_data_size = 0x00;
		nrf_receive_data_size = 0x00;
	}

	return status;
}

//������ ��������
uint8_t NRF_READ_REGISTER(uint8_t address, uint8_t size, uint8_t *value)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t i;

	//�������� ����������
	if(value == NULL)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//��������� ������� ������ �� ��������
		nrf_data_out[0x00] = NRF_SPI_CMD_R_REGISTER | (address & NRF_REGISTER_ADDRESS_MASK);
		for(i = 0x00; i < size; i++)
		{
			nrf_data_out[i + 0x01] = 0x00;
		}

		//������ �������
		status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, size + 0x01);

		//�������� �������
		if(status == NRF_ERROR_CODE_SUCCESS)
		{
			//��������� �����
			nrf_status_register = nrf_data_in[0x00];
			for(i = 0x00; i < size; i++)
			{
				value[i] = nrf_data_in[i + 0x01];
			}
		}
	}

	return status;
}

//������ ��������
uint8_t NRF_WRITE_REGISTER(uint8_t address, uint8_t size, uint8_t *value)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t i;

	//�������� ����������
	if(value == NULL)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//��������� ������� ������ � �������
		nrf_data_out[0x00] = NRF_SPI_CMD_W_REGISTER | (address & NRF_REGISTER_ADDRESS_MASK);
		for(i = 0x00; i < size; i++)
		{
			nrf_data_out[i + 0x01] = value[i];
		}

		//����� � �������
		status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, size + 0x01);

		//�������� �������
		if(status == NRF_ERROR_CODE_SUCCESS)
		{
			//��������� �����
			nrf_status_register = nrf_data_in[0x00];
		}
	}

	return status;
}

//������ �������� ������
uint8_t NRF_READ_RX_PAYLOAD(uint8_t size, uint8_t *data)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t i;

	//�������� ����������
	if(data == NULL)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//��������� ������� ������ ������
		nrf_data_out[0x00] = NRF_SPI_R_RX_PAYLOAD;
		for(i = 0x00; i < size; i++)
		{
			nrf_data_out[i + 0x01] = 0x00;
		}

		//����� �������
		status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, size + 0x01);

		//�������� �������
		if(status == NRF_ERROR_CODE_SUCCESS)
		{
			//��������� �����
			nrf_status_register = nrf_data_in[0x00];
			for(i = 0x00; i < size; i++)
			{
				data[i] = nrf_data_in[i + 0x01];
			}
		}
	}

	return status;
}

//������ ������ �� ��������
uint8_t NRF_WRITE_TX_PAYLOAD(uint8_t size, uint8_t *data)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t i;

	//�������� ����������
	if(data == NULL)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//��������� ������� ������ ������
		nrf_data_out[0x00] = NRF_SPI_W_TX_PAYLOAD;
		for(i = 0x00; i < size; i++)
		{
			nrf_data_out[i + 0x01] = data[i];
		}

		//����� �������
		status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, size + 0x01);

		//�������� �������
		if(status == NRF_ERROR_CODE_SUCCESS)
		{
			//��������� �����
			nrf_status_register = nrf_data_in[0x00];
		}
	}

	return status;
}

//�������� TX FIFO
uint8_t NRF_FLUSH_TX(void)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;

	//��������� �������
	nrf_data_out[0x00] = NRF_SPI_FLUSH_TX;

	//����� �������
	status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, 0x01);

	//�������� �������
	if(status == NRF_ERROR_CODE_SUCCESS)
	{
		//��������� �����
		nrf_status_register = nrf_data_in[0x00];
	}

	return status;
}

//�������� RX FIFO
uint8_t NRF_FLUSH_RX(void)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;

	//��������� �������
	nrf_data_out[0x00] = NRF_SPI_FLUSH_RX;

	//����� �������
	status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, 0x01);

	//�������� �������
	if(status == NRF_ERROR_CODE_SUCCESS)
	{
		//��������� �����
		nrf_status_register = nrf_data_in[0x00];
	}

	return status;
}

//��������� ������������� ��������� ���������� ������
uint8_t NRF_REUSE_TX_PAYLOAD(void)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;

	//��������� �������
	nrf_data_out[0x00] = NRF_SPI_REUSE_TX_PL;

	//����� �������
	status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, 0x01);

	//�������� �������
	if(status == NRF_ERROR_CODE_SUCCESS)
	{
		//��������� �����
		nrf_status_register = nrf_data_in[0x00];
	}

	return status;
}

//������ ������� ������ �� ����� FIFO
uint8_t NRF_READ_RX_PAYLOAD_WIDTH(uint8_t *width)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;

	//�������� ����������
	if(width == NULL)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//��������� ������� ������ ������ ������
		nrf_data_out[0x00] = NRF_SPI_R_RX_PL_WID;
		nrf_data_out[0x01] = 0x00;

		//����� �������
		status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, 0x02);

		//�������� �������
		if(status == NRF_ERROR_CODE_SUCCESS)
		{
			//��������� �����
			nrf_status_register = nrf_data_in[0x00];
			*width = nrf_data_in[0x01];
		}
	}

	return status;
}

//������ ������ ��� ������ ACK
uint8_t NRF_WRITE_ACK_PAYLOAD(uint8_t pipe, uint8_t size, uint8_t *data)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t i;

	//�������� ����������
	if(data == NULL)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//��������� ������� ������ ������
		nrf_data_out[0x00] = NRF_SPI_W_ACK_PAYLOAD | (pipe & NRF_REGISTER_PIPE_MASK);
		for(i = 0x00; i < size; i++)
		{
			nrf_data_out[i + 0x01] = data[i];
		}

		//����� �������
		status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, size + 0x01);

		//�������� �������
		if(status == NRF_ERROR_CODE_SUCCESS)
		{
			//��������� �����
			nrf_status_register = nrf_data_in[0x00];
		}
	}

	return status;
}

//������ ������ �� �������� ��� �������������
uint8_t NRF_WRITE_TX_PAYLOAD_NO_ACK(uint8_t size, uint8_t *data)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t i;

	//�������� ����������
	if(data == NULL)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//��������� ������� ������ ������
		nrf_data_out[0x00] = NRF_SPI_W_TX_PAYLOAD_NO_ACK;
		for(i = 0x00; i < size; i++)
		{
			nrf_data_out[i + 0x01] = data[i];
		}

		//����� �������
		status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, size + 0x01);

		//�������� �������
		if(status == NRF_ERROR_CODE_SUCCESS)
		{
			//��������� �����
			nrf_status_register = nrf_data_in[0x00];
		}
	}

	return status;
}

//������� NOP
uint8_t NRF_NOP(void)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;

	//��������� �������
	nrf_data_out[0x00] = NRF_SPI_NOP;

	//����� �������
	status = NRF_HAL_SPI_EXCHANGE(nrf_data_out, nrf_data_in, 0x01);

	//�������� �������
	if(status == NRF_ERROR_CODE_SUCCESS)
	{
		//��������� �����
		nrf_status_register = nrf_data_in[0x00];
	}

	return status;
}

//������ ������ �� ����
uint8_t NRF_WRITE_RX_ADDRESS(uint8_t pipe, uint8_t *addr, uint8_t size)
{
	uint8_t status;
	uint8_t reg_address;

	//�������� ����������
	if((pipe > NRF_RX_DATA_PIPE_NUMBER_5) || (size != nrf_size_address))
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//������� ����� �������� ��� ������ ����� � ����������� �� ������ PIPE
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

//������ ������ �� ��������
uint8_t NRF_WRITE_TX_ADDRESS(uint8_t *addr, uint8_t size)
{
	uint8_t status;

	//�������� ����������
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

//������� �����������, 0 - 125
uint8_t NRF_SET_CHANNEL(uint8_t channel)
{
	uint8_t status;

	//�������� ����������
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

//������� ���������� ��������� �������� �������
uint8_t NRF_SET_RETRANSMIT(uint8_t ard, uint8_t arc)
{
	uint8_t status;
	uint8_t reg_value;

	//�������� ����������
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

//������ ������ CRC, 1/2
uint8_t NRF_SET_CRC_SIZE(uint8_t size)
{
	uint8_t status;
	uint8_t reg_value;

	//�������� ����������
	if((size != NRF_CRC_LENGTH_1_BYTE) && (size != NRF_CRC_LENGTH_2_BYTE))
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//������� ���
		nrf_config &= ~NRF_REGISTER_CONFIG_CRCO;

		//�������� ����
		nrf_config |= NRF_REGISTER_CONFIG_EN_CRC;
		if(size == NRF_CRC_LENGTH_2_BYTE) nrf_config |= NRF_REGISTER_CONFIG_CRCO;
		status = NRF_WRITE_REGISTER(NRF_REGISTER_CONFIG, 0x01, &nrf_config);
	}

	return status;
}

//������ �������� ������������
uint8_t NRF_SET_POWER(uint8_t power)
{
	uint8_t status;

	//�������� ����������
	if(power > NRF_RF_PWR_0DBM)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//����� ��� ������� ��������
		nrf_power_and_baudrate &= ~NRF_REGISTER_RF_SETUP_RF_PWR_MASK;

		//���� ����� �������� ��������
		nrf_power_and_baudrate = (power << NRF_RF_PWR_BIT_LEFT_SHIFT);
		status = NRF_WRITE_REGISTER(NRF_REGISTER_RF_SETUP, 0x01, &nrf_power_and_baudrate);
	}

	return status;
}

//������ �������� ������
uint8_t NRF_SET_BAUDRATE(uint8_t baudrate)
{
	uint8_t status;

	//�������� ����������
	if(baudrate > NRF_DATA_RATE_2M)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//����� �������� ��������
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

//������ ������ ������ �� ���� ��� ��������� PIPE, �� 0 (�� ������������) �� 32, ��� ������ Static Length
uint8_t NRF_SET_RX_PAYLOAD_LENGTH(uint8_t pipe, uint8_t length)
{
	uint8_t status;
	uint8_t reg_address;

	//�������� ����������
	if((pipe > NRF_RX_DATA_PIPE_NUMBER_5) || (length > NRF_RF_MAXIMUM_DATA_EXCHANGE_SIZE))
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//������� ����� �������� ����� ����������� ������ � ����������� �� ������ PIPE
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

//��������� PIPE, ��������� ����, ��������� �����, ��������� ������������ ����� ������, ������ � ������
uint8_t NRF_SET_PIPE(uint8_t pipe, uint8_t en_pipe, uint8_t en_ack, uint8_t en_dpl, uint8_t en_ack_payload)
{
	uint8_t status;

	//�������� ����������
	if(pipe > NRF_RX_DATA_PIPE_NUMBER_5)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//�������� PIPE
		if(en_pipe == NRF_STATE_ON)
		{
			//���� ��������
			nrf_rxaddr |= (NRF_REGISTER_EN_RXADDR_ERX_0 << pipe);
		}
		else
		{
			//���� ��������
			nrf_rxaddr &= ~(NRF_REGISTER_EN_RXADDR_ERX_0 << pipe);
		}

		//����� � �������
		status = NRF_WRITE_REGISTER(NRF_REGISTER_EN_RXADDR, 0x01, &nrf_rxaddr);

		//�������� �� ACK ��� PIPE
		if(en_ack == NRF_STATE_ON)
		{
			//���� ��������
			nrf_enaa |= (NRF_REGISTER_EN_AA_ENAA_P0 << pipe);
		}
		else
		{
			//���� ��������
			nrf_enaa &= ~(NRF_REGISTER_EN_AA_ENAA_P0 << pipe);
		}

		//����� � �������
		status = NRF_WRITE_REGISTER(NRF_REGISTER_EN_AA, 0x01, &nrf_enaa);

		//��������� �� ������������ ����� ������
		if(en_dpl == NRF_STATE_ON)
		{
			//���� ��������
			nrf_dynpd |= (NRF_REGISTER_DYNPD_DPL_P0 << pipe);
		}
		else
		{
			//���� ��������
			nrf_dynpd &= ~(NRF_REGISTER_DYNPD_DPL_P0 << pipe);
		}

		//��������� �� ������ � ������ ACK
		if(en_ack_payload == NRF_STATE_ON)
		{
			//���� ��������
			//��� �� ������� EN_DYN_ACK
			//��� �� ����� �������� ��� ����� ACK ��� DYN_PD0
			nrf_feature |= NRF_REGISTER_FEATURE_EN_ACK_PAY;
			nrf_feature |= NRF_REGISTER_FEATURE_EN_DYN_ACK;
			nrf_dynpd |= NRF_REGISTER_DYNPD_DPL_P0;
		}
		else
		{
			//���� ��������
			//��� �� �������� EN_DYN_ACK
			nrf_feature &= ~NRF_REGISTER_FEATURE_EN_ACK_PAY;
			nrf_feature &= ~NRF_REGISTER_FEATURE_EN_DYN_ACK;
		}

		//����� � �������
		status = NRF_WRITE_REGISTER(NRF_REGISTER_DYNPD, 0x01, &nrf_dynpd);

		//���� ��������� ���� ���� ��� NRF_REGISTER_DYNPD, �� ����� �������� ��� EN_DPL �������� NRF_REGISTER_FEATURE
		//���� ����, �� ���������
		if(nrf_dynpd != 0x00)
		{
			//�������
			nrf_feature |= NRF_REGISTER_FEATURE_EN_DPL;
		}
		else
		{
			//��������
			nrf_feature &= ~NRF_REGISTER_FEATURE_EN_DPL;
		}

		//����� � �������
		status = NRF_WRITE_REGISTER(NRF_REGISTER_FEATURE, 0x01, &nrf_feature);
	}

	return status;
}

//������ �������� ������
uint8_t NRF_START_TRANSMIT(uint8_t *data, uint8_t size, uint32_t timeout, uint8_t no_ack_flag)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t reg_value;
	uint8_t i;

	//�������� ����������
	if((data == NULL) || (size == 0x00) || (timeout == 0x00))
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//�������� �������� ���������
		if(nrf_machine_state != NRF_MACHINE_STATE_FREE)
		{
			//������� ��������� �����
			return NRF_ERROR_CODE_CHANNEL_BUSY;
		}
		else
		{
			//����� CE � 0
			NRF_HAL_SET_CE_PIN_STATE(NRF_CE_STATE_OFF);

			//�������� ��������� ��������
			nrf_machine_state = NRF_MACHINE_STATE_START_SEND;

			//������ �������� �� �������� ���������� ��������
			NRF_HAL_START_TIMEOUT(NRF_TIMER_TIMEOUT, timeout);

			//�������� ���������������, PWR_UP = 1
			nrf_config |= NRF_REGISTER_CONFIG_PWR_UP;
			status = NRF_WRITE_REGISTER(NRF_REGISTER_CONFIG, 0x01, &nrf_config);

			//������ �������� ��� ����� �� ����� ���������
			NRF_HAL_START_TIMEOUT(NRF_TIMER_DELAY, NRF_TIME_POWER_UP);

			//�������� ������ ��� ��������
			for(i = 0x00; i < size; i++)
			{
				nrf_transmit_data[i] = data[i];
			}
			nrf_transmit_data_size = size;

			//���� �������� ��� �������������
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

//������ ����� ������
uint8_t NRF_START_RECEIVE(uint8_t *data, uint8_t size, uint32_t timeout)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t reg_value;
	uint8_t i;

	//�������� ����������
	if(timeout == 0x00)
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//�������� �������� ���������
		if(nrf_machine_state != NRF_MACHINE_STATE_FREE)
		{
			//������� ��������� �����
			return NRF_ERROR_CODE_CHANNEL_BUSY;
		}
		else
		{
			//����� CE � 0
			NRF_HAL_SET_CE_PIN_STATE(NRF_CE_STATE_OFF);

			//�������� ��������� ��������
			nrf_machine_state = NRF_MACHINE_STATE_START_RECEIVE;

			//������ �������� �� �������� ���������� ��������
			NRF_HAL_START_TIMEOUT(NRF_TIMER_TIMEOUT, timeout);

			//�������� ���������������, PWR_UP = 1
			nrf_config |= NRF_REGISTER_CONFIG_PWR_UP;
			status = NRF_WRITE_REGISTER(NRF_REGISTER_CONFIG, 0x01, &nrf_config);

			//������ �������� ��� ����� �� ����� ���������
			NRF_HAL_START_TIMEOUT(NRF_TIMER_DELAY, NRF_TIME_POWER_UP);

			//�������� ������ ��� �������� ACK
			for(i = 0x00; i < size; i++)
			{
				nrf_transmit_data[i] = data[i];
			}
			nrf_transmit_data_size = size;

			//����� ������� ������
			nrf_receive_data_size = 0x00;
		}
	}

	return status;
}

//�������� �������� ������
uint8_t NRF_GET_RECEIVED_DATA(uint8_t *data, uint8_t buffer_size, uint8_t *data_size)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t i;

	//�������� ����������
	if((data == NULL) || (data_size == NULL) || (buffer_size < nrf_receive_data_size))
	{
		return NRF_ERROR_CODE_INVALID_DATA_PARAMS;
	}
	else
	{
		//������� ������
		for(i = 0x00; i < nrf_receive_data_size; i++)
		{
			data[i] = nrf_receive_data[i];
		}
		*data_size = nrf_receive_data_size;

		//����� ������� ������
		nrf_receive_data_size = 0x00;
	}

	return status;
}

//��������� ����� � ��������
uint8_t NRF_EXCHANGE_PROCESSING(void)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t reg_value;
	uint8_t irq_state = NRF_STATE_OFF;

	//��������� ������� �� ��������� ��������

	//���� �������� �������� ������
	if(nrf_machine_state == NRF_MACHINE_STATE_START_SEND)
	{
		//�������� �������� ������

		//������� ��������� ����� ��������� ����������������
		if(NRF_HAL_STATUS_TIMEOUT(NRF_TIMER_DELAY) == NRF_ERROR_CODE_TIMEOUT)
		{
			//����� �������, ������ ��������� ��������
			nrf_machine_state = NRF_MACHINE_STATE_WAIT_SEND;

			//��������� ������ � FIFO �� ��������
			if(nrf_tx_no_ack_flag == NRF_STATE_ON)
			{
				//���� ��� �������������
				NRF_WRITE_TX_PAYLOAD_NO_ACK(nrf_transmit_data_size, nrf_transmit_data);
			}
			else
			{
				//���� � ��������������
				NRF_WRITE_TX_PAYLOAD(nrf_transmit_data_size, nrf_transmit_data);
			}

			//����� ������� ������
			nrf_transmit_data_size = 0x00;

			//�����
			NRF_HAL_MICRO_SECOND_DELAY(NRF_TIME_CE_START_TRANSMIT);

			//���������� ������ CE � 1
			NRF_HAL_SET_CE_PIN_STATE(NRF_CE_STATE_ON);

			//�����
			NRF_HAL_MICRO_SECOND_DELAY(NRF_TIME_CE_START_TRANSMIT);

			//���������� ������ CE � 0
			NRF_HAL_SET_CE_PIN_STATE(NRF_CE_STATE_OFF);

		    //��������� ������
			NRF_HAL_STOP_TIMEOUT(NRF_TIMER_DELAY);
		}
	}
	else
	//��� ���������� �������� ������
	if(nrf_machine_state == NRF_MACHINE_STATE_WAIT_SEND)
	{
		//��������� ������ ����������
		//���� ���� ����������, �� ������ ������ � �� ���� ������, ��� �� ����������
		//���� ��� ��������� ��������

		//������ ��������� ����������
		NRF_HAL_GET_IRQ_STATE(&irq_state);

		//������� ���� �� ����������
		if(irq_state == NRF_STATE_ON)
		{
			//���� ����������, ����� �������� ������� �������, ���������� ������� NOP
			NRF_NOP();

			//������� ���� ���������� � �������� �������

			//��� ���������� ���������� �������� ������
			if(nrf_status_register & NRF_REGISTER_STATUS_TX_DS_BIT)
			{
				//������� �������� ������
				//� ������� ����� ACK � �������

				//������ ����� ������ � ������� FIFO
				NRF_READ_RX_PAYLOAD_WIDTH(&nrf_receive_data_size);

				//���������� ������
				NRF_READ_RX_PAYLOAD(nrf_receive_data_size, nrf_receive_data);

				//��������� ����� ��� �������� ������
				status = NRF_STOP_PROCESSING();

				//�������� ������ ���������� ��������
				status = NRF_ERROR_CODE_SEND_DATA_SUCCESS;
			}

			//��� ���������� ������������� ����� ������� �������� ������
			if(nrf_status_register & NRF_REGISTER_STATUS_MAX_RT_BIT)
			{
				//������ ���������� ������������� ����� ������� �������� ������

				//��������� ����� ��� �������� ������
				status = NRF_STOP_PROCESSING();

				//�������� ������ ������
				status = NRF_ERROR_CODE_SEND_MAX_RT_ERROR;
			}
		}
		else
		{
			//�� ���� ����������, �������� �������
			if(NRF_HAL_STATUS_TIMEOUT(NRF_TIMER_TIMEOUT) == NRF_ERROR_CODE_TIMEOUT)
			{
				//������� ����

				//��������� ����� ��� �������� ������
				status = NRF_STOP_PROCESSING();

				//�������� ������ ��������
				status = NRF_ERROR_CODE_TIMEOUT;
			}
		}
	}
	else
	//��� ������� ���� ������
	if(nrf_machine_state == NRF_MACHINE_STATE_START_RECEIVE)
	{
		//������� ���� ������

		//������� ��������� ����� ��������� ����������������
		if(NRF_HAL_STATUS_TIMEOUT(NRF_TIMER_DELAY) == NRF_ERROR_CODE_TIMEOUT)
		{
			//����� �������, ������ ��������� ��������
			nrf_machine_state = NRF_MACHINE_STATE_WAIT_RECEIVE;

			//��������� ������ � FIFO �� �������� ACK, PIPE 1 ��� ����� ������
			NRF_WRITE_ACK_PAYLOAD(NRF_RX_DATA_PIPE_NUMBER_1, nrf_transmit_data_size, nrf_transmit_data);

			//�������� ��� PRIM_RX � 1
			nrf_config |= NRF_REGISTER_CONFIG_PWR_UP | NRF_REGISTER_CONFIG_PRIM_RX;
			status = NRF_WRITE_REGISTER(NRF_REGISTER_CONFIG, 0x01, &nrf_config);

			//�����
			NRF_HAL_MICRO_SECOND_DELAY(NRF_TIME_CE_START_RECEIVE);

			//���������� ������ CE � 1
			NRF_HAL_SET_CE_PIN_STATE(NRF_CE_STATE_ON);

		    //��������� ������
			NRF_HAL_STOP_TIMEOUT(NRF_TIMER_DELAY);
		}
	}
	else
	//��� ���������� ����� ������
	if(nrf_machine_state == NRF_MACHINE_STATE_WAIT_RECEIVE)
	{
		//��������� ������ ����������
		//���� ���� ����������, �� ������ ������ � �� ���� ������, ��� �� ����������
		//���� ��� ��������� ��������

		//������ ��������� ����������
		NRF_HAL_GET_IRQ_STATE(&irq_state);

		//������� ���� �� ����������
		if(irq_state == NRF_STATE_ON)
		{
			//���� ����������, ����� �������� ������� �������, ���������� ������� NOP
			NRF_NOP();

			//������� ���� ���������� � �������� �������

			//��� ���������� ����� ������
			if(nrf_status_register & NRF_REGISTER_STATUS_RX_DR_BIT)
			{
				//������� ������� ������

				//������ ����� ������ � ������� FIFO
				NRF_READ_RX_PAYLOAD_WIDTH(&nrf_receive_data_size);

				//���������� ������
				NRF_READ_RX_PAYLOAD(nrf_receive_data_size, nrf_receive_data);

				//��������� ����� ��� �������� ������
				status = NRF_STOP_PROCESSING();

				//�������� ������ ���������� �����
				status = NRF_ERROR_CODE_RECEIVE_DATA_SUCCESS;
			}
		}
		else
		{
			//�� ���� ����������, �������� �������
			if(NRF_HAL_STATUS_TIMEOUT(NRF_TIMER_TIMEOUT) == NRF_ERROR_CODE_TIMEOUT)
			{
				//������� ����

				//��������� ����� ��� �������� ������
				status = NRF_STOP_PROCESSING();

				//�������� ������ ��������
				status = NRF_ERROR_CODE_TIMEOUT;
			}
		}
	}

	return status;
}

//��������� ����� ��� �������� ������
uint8_t NRF_STOP_PROCESSING(void)
{
	uint8_t status = NRF_ERROR_CODE_SUCCESS;
	uint8_t reg_value;

	//��������� ������
	NRF_HAL_STOP_TIMEOUT(NRF_TIMER_TIMEOUT);

	//���������� ������ CE � 0
	NRF_HAL_SET_CE_PIN_STATE(NRF_CE_STATE_OFF);

	//����� �������� ��������
	reg_value = NRF_REGISTER_STATUS_RX_DR_BIT | NRF_REGISTER_STATUS_TX_DS_BIT | NRF_REGISTER_STATUS_MAX_RT_BIT;
	status = NRF_WRITE_REGISTER(NRF_REGISTER_STATUS, 0x01, &reg_value);

	//����� FIFO �� �������� � ����
	status = NRF_FLUSH_TX();
	status = NRF_FLUSH_RX();

	//�������� ���������������, PWR_UP = 0, PRIM_RX = 0
	nrf_config &= ~NRF_REGISTER_CONFIG_PWR_UP;
	nrf_config &= ~NRF_REGISTER_CONFIG_PRIM_RX;
	status = NRF_WRITE_REGISTER(NRF_REGISTER_CONFIG, 0x01, &nrf_config);

	//����� �������� ���������
	nrf_machine_state = NRF_MACHINE_STATE_FREE;

	return status;
}
