/*
 * NRF24L01PLUS.h
 *
 *  Created on: 23 ���. 2021 �.
 *      Author: K.Kuznetzov
 */

#ifndef NRF24L01PLUS_H_
#define NRF24L01PLUS_H_

#include "NRF24L01PLUS_DEFINES.h"

uint8_t NRF_INIT(uint8_t *tx_address, uint8_t *rx_address, uint8_t size_address);//������������� NRF24L01PLUS
uint8_t NRF_READ_REGISTER(uint8_t address, uint8_t size, uint8_t *value);//������ ��������
uint8_t NRF_WRITE_REGISTER(uint8_t address, uint8_t size, uint8_t *value);//������ ��������
uint8_t NRF_READ_RX_PAYLOAD(uint8_t size, uint8_t *data);//������ �������� ������
uint8_t NRF_WRITE_TX_PAYLOAD(uint8_t size, uint8_t *data);//������ ������ �� ��������
uint8_t NRF_FLUSH_TX(void);//�������� TX FIFO
uint8_t NRF_FLUSH_RX(void);//�������� RX FIFO
uint8_t NRF_REUSE_TX_PAYLOAD(void);//��������� ������������� ��������� ���������� ������
uint8_t NRF_READ_RX_PAYLOAD_WIDTH(uint8_t *width);//������ ������� ������ �� ����� FIFO
uint8_t NRF_WRITE_ACK_PAYLOAD(uint8_t pipe, uint8_t size, uint8_t *data);//������ ������ ��� ������ ACK
uint8_t NRF_WRITE_TX_PAYLOAD_NO_ACK(uint8_t size, uint8_t *data);//������ ������ �� �������� ��� �������������
uint8_t NRF_NOP(void);//������� NOP
uint8_t NRF_WRITE_RX_ADDRESS(uint8_t *addr, uint8_t size);//������ ������ �� ����
uint8_t NRF_WRITE_TX_ADDRESS(uint8_t *addr, uint8_t size);//������ ������ �� ��������

uint8_t NRF_START_TRANSMIT(uint8_t *data, uint8_t size, uint32_t timeout);//������ �������� ������
uint8_t NRF_START_RECEIVE(uint8_t *data, uint8_t size, uint32_t timeout);//������ ����� ������
uint8_t NRF_GET_RECEIVED_DATA(uint8_t *data, uint8_t *size);//�������� �������� ������
uint8_t NRF_EXCHANGE_PROCESSING(void);//��������� ����� � ��������
uint8_t NRF_STOP_PROCESSING(void);//��������� ����� ��� �������� ������

//-������� �� �������� � �� ���?
#endif /* NRF24L01PLUS_H_ */
