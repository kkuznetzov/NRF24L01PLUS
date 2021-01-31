/*
 * NRF24L01PLUS_HAL.h
 *
 *  Created on: 23 янв. 2021 г.
 *      Author: K.Kuznetzov
 */

#ifndef NRF24L01PLUS_HAL_H_
#define NRF24L01PLUS_HAL_H_

uint8_t NRF_HAL_INIT(void);//Инициализация HAL
uint8_t NRF_HAL_SPI_EXCHANGE(uint8_t *data_out, uint8_t *data_in, uint16_t size_data);//Обмен данными по SPI с приёмопередатчиком NRF24L01PLUS
uint8_t NRF_HAL_MICRO_SECOND_DELAY(uint32_t delay);//Пауза в микросеундах
uint8_t NRF_HAL_START_TIMEOUT(uint8_t number, uint32_t time);//Запуск таймаута
uint8_t NRF_HAL_STOP_TIMEOUT(uint8_t number);//Остановка таймаута
uint8_t NRF_HAL_STATUS_TIMEOUT(uint8_t number);//Статус таймаута
uint8_t NRF_HAL_SET_CE_PIN_STATE(uint8_t state);//Установить состояние вывода CE
uint8_t NRF_HAL_GET_IRQ_STATE(uint8_t *state);//Запросить состояние прерывания IRQ

#endif /* NRF24L01PLUS_HAL_H_ */
