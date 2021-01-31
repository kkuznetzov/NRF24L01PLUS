/*
 * NRF24L01PLUS_DEFINES.h
 *
 *  Created on: 23 янв. 2021 г.
 *      Author: K.Kuznetzov
 */

#ifndef NRF24L01PLUS_DEFINES_H_
#define NRF24L01PLUS_DEFINES_H_

#include <stdint.h>

//Состояния
#define NRF_STATE_ON     0xFF //Включено
#define NRF_STATE_OFF    0x00 //Выключено

//Состояния вывода CE
#define NRF_CE_STATE_ON  0xFF //Включено
#define NRF_CE_STATE_OFF 0x00 //Выключено

//Адреса регистров NRF24L01PLUS
#define NRF_REGISTER_CONFIG      0x00 //Адрес регистра Configuration Register
#define NRF_REGISTER_EN_AA       0x01 //Адрес регистра Enable 'Auto Acknowledgment' Function Disable
#define NRF_REGISTER_EN_RXADDR   0x02 //Адрес регистра Enabled RX Addresses
#define NRF_REGISTER_SETUP_AW    0x03 //Адрес регистра Setup of Address Width
#define NRF_REGISTER_SETUP_RETR  0x04 //Адрес регистра Setup of Automatic Retransmission
#define NRF_REGISTER_RF_CH       0x05 //Адрес регистра RF Channnel
#define NRF_REGISTER_RF_SETUP    0x06 //Адрес регистра RF Setup Register
#define NRF_REGISTER_STATUS      0x07 //Адрес регистра Status Register
#define NRF_REGISTER_OBSERVE_TX  0x08 //Адрес регистра Transmit Observe Register
#define NRF_REGISTER_RPD         0x09 //Адрес регистра Receive Power Detector
#define NRF_REGISTER_RX_ADDR_P0  0x0A //Адрес регистра Receive Address Data Pipe 0
#define NRF_REGISTER_RX_ADDR_P1  0x0B //Адрес регистра Receive Address Data Pipe 1
#define NRF_REGISTER_RX_ADDR_P2  0x0C //Адрес регистра Receive Address Data Pipe 2
#define NRF_REGISTER_RX_ADDR_P3  0x0D //Адрес регистра Receive Address Data Pipe 3
#define NRF_REGISTER_RX_ADDR_P4  0x0E //Адрес регистра Receive Address Data Pipe 4
#define NRF_REGISTER_RX_ADDR_P5  0x0F //Адрес регистра Receive Address Data Pipe 5
#define NRF_REGISTER_TX_ADDR     0x10 //Адрес регистра Transmit Address
#define NRF_REGISTER_RX_PW_P0    0x11 //Адрес регистра Number of bytes in RX payload in data pipe 0
#define NRF_REGISTER_RX_PW_P1    0x12 //Адрес регистра Number of bytes in RX payload in data pipe 1
#define NRF_REGISTER_RX_PW_P2    0x13 //Адрес регистра Number of bytes in RX payload in data pipe 2
#define NRF_REGISTER_RX_PW_P3    0x14 //Адрес регистра Number of bytes in RX payload in data pipe 3
#define NRF_REGISTER_RX_PW_P4    0x15 //Адрес регистра Number of bytes in RX payload in data pipe 4
#define NRF_REGISTER_RX_PW_P5    0x16 //Адрес регистра Number of bytes in RX payload in data pipe 5
#define NRF_REGISTER_FIFO_STATUS 0x17 //Адрес регистра FIFO Status Register
#define NRF_REGISTER_DYNPD       0x1C //Адрес регистра Enable dynamic payload length
#define NRF_REGISTER_FEATURE     0x1D //Адрес регистра Feature Register

//Команды интерфейса SPI
#define NRF_SPI_CMD_R_REGISTER      0x00 //Чтение регистра, адрес регистра это младшие 5 бит команды
#define NRF_SPI_CMD_W_REGISTER      0x20 //Запись регистра, адрес регистра это младшие 5 бит команды
#define NRF_SPI_R_RX_PAYLOAD        0x61 //Команда чтения принятых данных
#define NRF_SPI_W_TX_PAYLOAD        0xA0 //Команда записи данных на передачу
#define NRF_SPI_FLUSH_TX            0xE1 //Отчистка FIFO на передачу
#define NRF_SPI_FLUSH_RX            0xE2 //Отчистка FIFO на приём
#define NRF_SPI_REUSE_TX_PL         0xE3 //Повторное использование последних переданных данных
#define NRF_SPI_R_RX_PL_WID         0x60 //Чтение размера данных на верху FIFO
#define NRF_SPI_W_ACK_PAYLOAD       0xA8 //Запись данных для ответа ACK
#define NRF_SPI_W_TX_PAYLOAD_NO_ACK 0xB0 //Запись данных для передачи их без подтверждения ACK
#define NRF_SPI_NOP                 0xFF //Пустая команды

//Маска выделения адресов регистров
#define NRF_REGISTER_ADDRESS_MASK   0x1F

//Маска выделения канала (PIPE)
#define NRF_REGISTER_PIPE_MASK      0x07

//Максимальная длина данных передаваемых через SPI
#define NRF_SPI_MAXIMUM_DATA_EXCHANGE_SIZE 0x21 //1 байт команды + 32 байта данных

//Максимальная длина данных для передачи через радиоканал
#define NRF_RF_MAXIMUM_DATA_EXCHANGE_SIZE  0x20 //32 байта данных

//Первый (нулевой) байт прочитанный из обмена по SPI содержит регистр статуса
#define NRF_SPI_STATUS_DATA_BYTE_NUMBER    0x00

//Коды ошибок
#define NRF_ERROR_CODE_SUCCESS              0x00 //Нет ошибок
#define NRF_ERROR_CODE_INVALID_DATA_PARAMS  0x01 //Ошибочные параметры
#define NRF_ERROR_CODE_SPI_EXCHANGE_ERROR   0x02 //Ошибка обмена по SPI
#define NRF_ERROR_CODE_TIMEOUT              0x03 //Ошибка таймаут
#define NRF_ERROR_CODE_SEND_DATA_SUCCESS    0x04 //Успех передачи данных
#define NRF_ERROR_CODE_RECEIVE_DATA_SUCCESS 0x05 //Успех приёма данных
#define NRF_ERROR_CODE_CHANNEL_BUSY         0x06 //Радиоканал/чип занят
#define NRF_ERROR_CODE_SEND_MAX_RT_ERROR    0x07 //Ошибка превышения максимального числа попыток отправки данных

//Скорости обмена через радиоканал
#define NRF_DATA_RATE_250K          0x00 //Скорость обмена 250K
#define NRF_DATA_RATE_1M            0x01 //Скорость обмена 1000K = 1M
#define NRF_DATA_RATE_2M            0x02 //Скорость обмена 2000K = 2M

//Значение скоростей обмена
#define NRF_DATA_RATE_250K_VALUE    250000  //Скорость обмена 250K
#define NRF_DATA_RATE_1M_VALUE      1000000 //Скорость обмена 1000K
#define NRF_DATA_RATE_2M_VALUE      2000000 //Скорость обмена 2000K

//Значения мощностей передатчика
#define NRF_RF_PWR_MINUS_18DBM      0x00 //Мощность передатчика -18 дБм
#define NRF_RF_PWR_MINUS_12DBM      0x01 //Мощность передатчика -12 дБм
#define NRF_RF_PWR_MINUS_6DBM       0x02 //Мощность передатчика -6 дБм
#define NRF_RF_PWR_0DBM             0x03 //Мощность передатчика 0 дБм

//Смещение бит в регистре для задания скорости
#define NRF_RF_PWR_BIT_LEFT_SHIFT   0x01 //На один бит влево

//Длины CRC
#define NRF_CRC_LENGTH_1_BYTE       0x00 //Длина CRC 1 байт
#define NRF_CRC_LENGTH_2_BYTE       0x01 //Длина CRC 2 байта

//Длина адреса на передачу и приём, для записи в регистр
#define NRF_ADDRESS_LENGTH_3_BYTE   0x00 //Длина адреса 3 байта
#define NRF_ADDRESS_LENGTH_4_BYTE   0x01 //Длина адреса 4 байта
#define NRF_ADDRESS_LENGTH_5_BYTE   0x02 //Длина адреса 5 байта

//Длина адреса на передачу и приём, значение
#define NRF_ADDRESS_LENGTH_3_BYTE_VALUE  0x03 //Длина адреса 3 байта
#define NRF_ADDRESS_LENGTH_4_BYTE_VALUE  0x04 //Длина адреса 4 байта
#define NRF_ADDRESS_LENGTH_5_BYTE_VALUE  0x05 //Длина адреса 5 байта

//Минимальное и максимальное значение для задержки перед повторной передачи пакета, шаг в 1 - это 250 мкс, Auto Retransmit Delay
#define NRF_ARD_MINIMUM             0x00 //250 мкс
#define NRF_ARD_MAXIMUM             0x0F //4000 мкс

//Значение шага задержки перед повторной передачей
#define NRF_ARD_STEP_VALUE          250 //250 мкс

//Смещение бит в регистре, для задержки перед повторной передачей
#define NRF_ARD_BIT_LEFT_SHIFT     0x04 //На 4 бита влево

//Минимальное и максимальное значение для числа повторных передач пакетов
#define NRF_ARC_MINIMUM             0x00 //0 - без повторных передач
#define NRF_ARC_MAXIMUM             0x0F //15 - повторов

//Значения по умолчанию для задержки перед повторной отправкой пакетов и числа повторных отправок пакетов
#define NRF_ARD_DEFAULT             0x00 //0 - 250 мкс
#define NRF_ARC_DEFAULT             0x03 //3 - три отправки

//Минимальное и максимальное значения номера радиоканала, с шагом 1 МГц начиная с 2400 МГц
#define NRF_RADIO_CHANNEL_NUMBER_MINIMUM 0x00 //Канал 0
#define NRF_RADIO_CHANNEL_NUMBER_MAXIMUM 0x7F //Канал 127
#define NRF_RADIO_CHANNEL_DEFAULT        0x02 //Номер канала по умолчанию

//Ширина радиоканала и начальная частота
#define NRF_RADIO_CHANNEL_WIDTH 1    //1 МГц
#define NRF_RADIO_CHANNEL_START 2400 //2400 МГц

//Число каналов на приём и номера каналов
#define NRF_RX_DATA_PIPE_COUNT    6 //6 каналов на приём
#define NRF_RX_DATA_PIPE_NUMBER_0 0 //Канал 0
#define NRF_RX_DATA_PIPE_NUMBER_1 1 //Канал 1
#define NRF_RX_DATA_PIPE_NUMBER_2 2 //Канал 2
#define NRF_RX_DATA_PIPE_NUMBER_3 3 //Канал 3
#define NRF_RX_DATA_PIPE_NUMBER_4 4 //Канал 4
#define NRF_RX_DATA_PIPE_NUMBER_5 5 //Канал 5

//Смещение бит в регистре статуса для номера канала
#define NRF_RX_DATA_PIPE_NUMBER_LEFT_SHIFT 1 //На 1 бит влево

//Значения бит для пустого FIFO на приём в регистре статуса
#define NRF_RX_DATA_PIPE_RF_FIFO_EMPTY 0x0E

//Смещение бит для выделения числа потеряных пакетов
#define NRF_COUTN_LOST_PACKET_BIT_LEFT_SHIFT 0x04 //Смещение на 4 бита влево

//Битовые значения/маски для регистра CONIG
#define NRF_REGISTER_CONFIG_MASK_RX_DR  0x40 //Бит запрета прерывания на приём, 1 - запрещено, 0 - разрешено
#define NRF_REGISTER_CONFIG_MASK_TX_DS  0x20 //Бит запрета прерывания на передачу, 1 - запрещено, 0 - разрешено
#define NRF_REGISTER_CONFIG_MASK_MAX_RT 0x10 //Бит запрета прерывания на максимальное число перезапросов, 1 - запрещено, 0 - разрешено
#define NRF_REGISTER_CONFIG_EN_CRC      0x08 //Бит разрешения CRC, 1- разрешено
#define NRF_REGISTER_CONFIG_CRCO        0x04 //Бит длины CRC, 0 - 1 байт, 1 - 2 байта
#define NRF_REGISTER_CONFIG_PWR_UP      0x02 //Бит включения, 1 - POWER UP, 0 - POWER DOWN
#define NRF_REGISTER_CONFIG_PRIM_RX     0x01 //Бит управления режимом приём/передача, 1 - приём, 0 - передача

//Битовые значения/маски для регистра EN_AA
#define NRF_REGISTER_EN_AA_ENAA_P5      0x20 //Бит разрешения подтверждения пакета для канала 5
#define NRF_REGISTER_EN_AA_ENAA_P4      0x10 //Бит разрешения подтверждения пакета для канала 4
#define NRF_REGISTER_EN_AA_ENAA_P3      0x08 //Бит разрешения подтверждения пакета для канала 3
#define NRF_REGISTER_EN_AA_ENAA_P2      0x04 //Бит разрешения подтверждения пакета для канала 2
#define NRF_REGISTER_EN_AA_ENAA_P1      0x02 //Бит разрешения подтверждения пакета для канала 1
#define NRF_REGISTER_EN_AA_ENAA_P0      0x01 //Бит разрешения подтверждения пакета для канала 0

//Битовые значения/маски для регистра EN_RXADDR
#define NRF_REGISTER_EN_RXADDR_ERX_5    0x20 //Бит разрешения приёмного канала 5
#define NRF_REGISTER_EN_RXADDR_ERX_4    0x10 //Бит разрешения приёмного канала 4
#define NRF_REGISTER_EN_RXADDR_ERX_3    0x08 //Бит разрешения приёмного канала 3
#define NRF_REGISTER_EN_RXADDR_ERX_2    0x04 //Бит разрешения приёмного канала 2
#define NRF_REGISTER_EN_RXADDR_ERX_1    0x02 //Бит разрешения приёмного канала 1
#define NRF_REGISTER_EN_RXADDR_ERX_0    0x01 //Бит разрешения приёмного канала 0

//Битовые значения/маски для регистра SETUP_AW
#define NRF_REGISTER_SETUP_AW_3_BYTES   0x01 //Длина адреса 3 байта
#define NRF_REGISTER_SETUP_AW_4_BYTES   0x02 //Длина адреса 4 байта
#define NRF_REGISTER_SETUP_AW_5_BYTES   0x03 //Длина адреса 5 байт

//Битовые значения/маски для регистра SETUP_RETR
#define NRF_REGISTER_SETUP_RETR_ARD_MASK 0xF0 //Маска бит для задержки перед повторной передачей пакета, Auto Retransmit Delay, шаг 250 мкс
#define NRF_REGISTER_SETUP_RETR_ARC_MASK 0x0F //Маска бит числа повторных передач пакетов

//Битовые значения/маски для регистра SETUP_RETR
#define NRF_REGISTER_RF_CH_MASK          0x7F //Маска для задания радиоканала

//Битовые значения/маски для регистра RF_SETUP
#define NRF_REGISTER_RF_SETUP_CONT_WAVE   0x80 //Бит включения непрерывного сигнала несущей на выходе
#define NRF_REGISTER_RF_SETUP_RF_DR_LOW   0x20 //Бит скорости Data Rate в 250К, бит задания скорости
#define NRF_REGISTER_RF_SETUP_PLL_LOCK    0x10 //Бит PLL Lock для тестирования
#define NRF_REGISTER_RF_SETUP_RF_DR_HIGH  0x08 //Бит скорости Data Rate, бит задания скорости
#define NRF_REGISTER_RF_SETUP_RF_PWR_MASK 0x06 //Маска бит для задания мощности передатчика

//Битовые значения/маски для регистра STATUS
#define NRF_REGISTER_STATUS_RX_DR_BIT     0x40 //Бит прерывания Data Ready RX FIFO interrupt
#define NRF_REGISTER_STATUS_TX_DS_BIT     0x20 //Бит прерывания Data Sent TX FIFO interrupt
#define NRF_REGISTER_STATUS_MAX_RT_BIT    0x10 //Бит прерывания Maximum number of TX retransmits interrupt
#define NRF_REGISTER_STATUS_RX_P_NO_MASK  0x0E //Маска бит для номера канала принявшего данные в FIFO
#define NRF_REGISTER_STATUS_TX_FULL_BIT   0x01 //Бит заполнености FIFO на передачу

//Битовые значения/маски для регистра OBSERVE_TX
#define NRF_REGISTER_OBSERVE_TX_PLOS_CNT_MASK 0xF0 //Маска для числа потеряных пакетов
#define NRF_REGISTER_OBSERVE_TX_ARC_CNT_MASK  0x0F //Маска для числа переданных повторно пакетов

//Битовые значения/маски для регистра RPD
#define NRF_REGISTER_RPD_RX_POWER_DETECTOR_BIT 0x01 //Бит превышения порога примаемым сигналом, порог -64 дБм

//Битовые значения/маски для регистров RX_PW_P0 - RX_PW_P5
#define NRF_REGISTER_RX_PW_P0_P5_MASK     0x3F//Маска выделения числа принятых байт каналом (PIPE)

//Битовые значения/маски для регистра FIFO_STATUS
#define NRF_REGISTER_FIFO_STATUS_TX_REUSE_BIT 0x40 //Бит повторного использования последней отправленой посылки
#define NRF_REGISTER_FIFO_STATUS_TX_FULL      0x20 //Флаг заполнености FIFO на передачу
#define NRF_REGISTER_FIFO_STATUS_TX_EMPTY     0x10 //Флаг пустоты FIFO на передачу
#define NRF_REGISTER_FIFO_STATUS_RX_FULL      0x02 //Флаг заполнености FIFO на приём
#define NRF_REGISTER_FIFO_STATUS_RX_EMPTY     0x01 //Флаг пустоты FIFO на приём

//Битовые значения/маски для регистра DYNPD
#define NRF_REGISTER_DYNPD_DPL_P5   0x20 //Бит разрешения динамической длины посылки для канала (PIPE) 5
#define NRF_REGISTER_DYNPD_DPL_P4   0x10 //Бит разрешения динамической длины посылки для канала (PIPE) 4
#define NRF_REGISTER_DYNPD_DPL_P3   0x08 //Бит разрешения динамической длины посылки для канала (PIPE) 3
#define NRF_REGISTER_DYNPD_DPL_P2   0x04 //Бит разрешения динамической длины посылки для канала (PIPE) 2
#define NRF_REGISTER_DYNPD_DPL_P1   0x02 //Бит разрешения динамической длины посылки для канала (PIPE) 1
#define NRF_REGISTER_DYNPD_DPL_P0   0x01 //Бит разрешения динамической длины посылки для канала (PIPE) 0

//Битовые значения/маски для регистра FEATURE
#define NRF_REGISTER_FEATURE_EN_DPL     0x04 //Бит разрешения динамической длины посылки
#define NRF_REGISTER_FEATURE_EN_ACK_PAY 0x02 //Бит разрешения передачи данных с подтверждением ACK
#define NRF_REGISTER_FEATURE_EN_DYN_ACK 0x01 //Бит разрешения команды NRF_SPI_W_TX_PAYLOAD_NO_AC

//Число таймеров/таймаутов и их описание
#define NRF_TIMERS_COUNT  0x02 //Два таймера
#define NRF_TIMER_DELAY   0x00 //Для пауз
#define NRF_TIMER_TIMEOUT 0x01 //Для таймаутов

//Значение пауз и таймаутов
#define NRF_TIME_POWER_UP          0x03 //3 мс с запасом, идеально 1.5 мс
#define NRF_TIME_CE_START_TRANSMIT 0x0E //15 мкс с запасом, достаточно 10 мкс
#define NRF_TIME_CE_START_RECEIVE  0x0E //Пауза перед приёмом

//Состояния автомата реализующего обмен через радиоканал
#define NRF_MACHINE_STATE_FREE          0x00 //Автомат состояний свободен
#define NRF_MACHINE_STATE_START_SEND    0x01 //Состояние начала передачи
#define NRF_MACHINE_STATE_WAIT_SEND     0x02 //Состояние ожидания завершения передачи
#define NRF_MACHINE_STATE_START_RECEIVE 0x03 //Состояние начала приёма
#define NRF_MACHINE_STATE_WAIT_RECEIVE  0x04 //Состояние ожидания завершения приёма

#endif /* NRF24L01PLUS_DEFINES_H_ */
