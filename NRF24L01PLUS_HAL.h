/*
 * NRF24L01PLUS_HAL.h
 *
 *  Created on: 23 ���. 2021 �.
 *      Author: K.Kuznetzov
 */

#ifndef NRF24L01PLUS_HAL_H_
#define NRF24L01PLUS_HAL_H_

uint8_t NRF_HAL_INIT(void);//������������� HAL
uint8_t NRF_HAL_SPI_EXCHANGE(uint8_t *data_out, uint8_t *data_in, uint16_t size_data);//����� ������� �� SPI � ����������������� NRF24L01PLUS
uint8_t NRF_HAL_MICRO_SECOND_DELAY(uint32_t delay);//����� � ������������
uint8_t NRF_HAL_START_TIMEOUT(uint8_t number, uint32_t time);//������ ��������
uint8_t NRF_HAL_STOP_TIMEOUT(uint8_t number);//��������� ��������
uint8_t NRF_HAL_STATUS_TIMEOUT(uint8_t number);//������ ��������
uint8_t NRF_HAL_SET_CE_PIN_STATE(uint8_t state);//���������� ��������� ������ CE
uint8_t NRF_HAL_GET_IRQ_STATE(uint8_t *state);//��������� ��������� ���������� IRQ

#endif /* NRF24L01PLUS_HAL_H_ */
