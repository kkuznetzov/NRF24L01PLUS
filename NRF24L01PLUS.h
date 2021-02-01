/*
 * NRF24L01PLUS.h
 *
 *  Created on: 23 янв. 2021 г.
 *      Author: K.Kuznetzov
 */

#ifndef NRF24L01PLUS_H_
#define NRF24L01PLUS_H_

#include "NRF24L01PLUS_DEFINES.h"

uint8_t NRF_INIT(uint8_t size_address);//Инициализация NRF24L01PLUS
uint8_t NRF_READ_REGISTER(uint8_t address, uint8_t size, uint8_t *value);//Чтение регистра
uint8_t NRF_WRITE_REGISTER(uint8_t address, uint8_t size, uint8_t *value);//Запись регистра
uint8_t NRF_READ_RX_PAYLOAD(uint8_t size, uint8_t *data);//Чтение принятых данных
uint8_t NRF_WRITE_TX_PAYLOAD(uint8_t size, uint8_t *data);//Запись данных на передачу
uint8_t NRF_FLUSH_TX(void);//Отчистка TX FIFO
uint8_t NRF_FLUSH_RX(void);//Отчистка RX FIFO
uint8_t NRF_REUSE_TX_PAYLOAD(void);//Повторное использование последних переданных данных
uint8_t NRF_READ_RX_PAYLOAD_WIDTH(uint8_t *width);//Чтение размера данных на верху FIFO
uint8_t NRF_WRITE_ACK_PAYLOAD(uint8_t pipe, uint8_t size, uint8_t *data);//Запись данных для ответа ACK
uint8_t NRF_WRITE_TX_PAYLOAD_NO_ACK(uint8_t size, uint8_t *data);//Запись данных на передачу без подтверждения
uint8_t NRF_NOP(void);//Команда NOP
uint8_t NRF_WRITE_RX_ADDRESS(uint8_t pipe, uint8_t *addr, uint8_t size);//Запись адреса на приём
uint8_t NRF_WRITE_TX_ADDRESS(uint8_t *addr, uint8_t size);//Запись адреса на передачу
uint8_t NRF_SET_CHANNEL(uint8_t channel);//Задание радиоканала, 0 - 125
uint8_t NRF_SET_RETRANSMIT(uint8_t ard, uint8_t arc);//Задание параметров повторной передачи пакетов
uint8_t NRF_SET_CRC_SIZE(uint8_t size);//Задать размер CRC, 1/2
uint8_t NRF_SET_POWER(uint8_t power);//Задать мощность передачтчика
uint8_t NRF_SET_BAUDRATE(uint8_t baudrate);//Задать скорость обмена
uint8_t NRF_SET_RX_PAYLOAD_LENGTH(uint8_t pipe, uint8_t length);//Задать размер данных на приём для заданного PIPE, от 0 (не используется) до 32, для режима Static Length
uint8_t NRF_SET_PIPE(uint8_t pipe, uint8_t en_pipe, uint8_t en_ack, uint8_t en_dpl);//Настройка PIPE, разрешить приём, разрешить ответ, разрешить динамическую длину ответа

uint8_t NRF_START_TRANSMIT(uint8_t *data, uint8_t size, uint32_t timeout, uint8_t no_ack_flag);//Запуск передачи пакета
uint8_t NRF_START_RECEIVE(uint8_t *data, uint8_t size, uint32_t timeout);//Запуск приёма пакета
uint8_t NRF_GET_RECEIVED_DATA(uint8_t *data, uint8_t *size);//Получить принятые данные
uint8_t NRF_EXCHANGE_PROCESSING(void);//Обработка приёма и передачи
uint8_t NRF_STOP_PROCESSING(void);//Остановка приёма или передачи данных

//-таймаут на передачу и на прём?
#endif /* NRF24L01PLUS_H_ */
