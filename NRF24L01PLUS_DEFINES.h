/*
 * NRF24L01PLUS_DEFINES.h
 *
 *  Created on: 23 ���. 2021 �.
 *      Author: K.Kuznetzov
 */

#ifndef NRF24L01PLUS_DEFINES_H_
#define NRF24L01PLUS_DEFINES_H_

#include <stdint.h>

//���������
#define NRF_STATE_ON     0xFF //��������
#define NRF_STATE_OFF    0x00 //���������

//��������� ������ CE
#define NRF_CE_STATE_ON  0xFF //��������
#define NRF_CE_STATE_OFF 0x00 //���������

//������ ��������� NRF24L01PLUS
#define NRF_REGISTER_CONFIG      0x00 //����� �������� Configuration Register
#define NRF_REGISTER_EN_AA       0x01 //����� �������� Enable 'Auto Acknowledgment' Function Disable
#define NRF_REGISTER_EN_RXADDR   0x02 //����� �������� Enabled RX Addresses
#define NRF_REGISTER_SETUP_AW    0x03 //����� �������� Setup of Address Width
#define NRF_REGISTER_SETUP_RETR  0x04 //����� �������� Setup of Automatic Retransmission
#define NRF_REGISTER_RF_CH       0x05 //����� �������� RF Channnel
#define NRF_REGISTER_RF_SETUP    0x06 //����� �������� RF Setup Register
#define NRF_REGISTER_STATUS      0x07 //����� �������� Status Register
#define NRF_REGISTER_OBSERVE_TX  0x08 //����� �������� Transmit Observe Register
#define NRF_REGISTER_RPD         0x09 //����� �������� Receive Power Detector
#define NRF_REGISTER_RX_ADDR_P0  0x0A //����� �������� Receive Address Data Pipe 0
#define NRF_REGISTER_RX_ADDR_P1  0x0B //����� �������� Receive Address Data Pipe 1
#define NRF_REGISTER_RX_ADDR_P2  0x0C //����� �������� Receive Address Data Pipe 2
#define NRF_REGISTER_RX_ADDR_P3  0x0D //����� �������� Receive Address Data Pipe 3
#define NRF_REGISTER_RX_ADDR_P4  0x0E //����� �������� Receive Address Data Pipe 4
#define NRF_REGISTER_RX_ADDR_P5  0x0F //����� �������� Receive Address Data Pipe 5
#define NRF_REGISTER_TX_ADDR     0x10 //����� �������� Transmit Address
#define NRF_REGISTER_RX_PW_P0    0x11 //����� �������� Number of bytes in RX payload in data pipe 0
#define NRF_REGISTER_RX_PW_P1    0x12 //����� �������� Number of bytes in RX payload in data pipe 1
#define NRF_REGISTER_RX_PW_P2    0x13 //����� �������� Number of bytes in RX payload in data pipe 2
#define NRF_REGISTER_RX_PW_P3    0x14 //����� �������� Number of bytes in RX payload in data pipe 3
#define NRF_REGISTER_RX_PW_P4    0x15 //����� �������� Number of bytes in RX payload in data pipe 4
#define NRF_REGISTER_RX_PW_P5    0x16 //����� �������� Number of bytes in RX payload in data pipe 5
#define NRF_REGISTER_FIFO_STATUS 0x17 //����� �������� FIFO Status Register
#define NRF_REGISTER_DYNPD       0x1C //����� �������� Enable dynamic payload length
#define NRF_REGISTER_FEATURE     0x1D //����� �������� Feature Register

//������� ���������� SPI
#define NRF_SPI_CMD_R_REGISTER      0x00 //������ ��������, ����� �������� ��� ������� 5 ��� �������
#define NRF_SPI_CMD_W_REGISTER      0x20 //������ ��������, ����� �������� ��� ������� 5 ��� �������
#define NRF_SPI_R_RX_PAYLOAD        0x61 //������� ������ �������� ������
#define NRF_SPI_W_TX_PAYLOAD        0xA0 //������� ������ ������ �� ��������
#define NRF_SPI_FLUSH_TX            0xE1 //�������� FIFO �� ��������
#define NRF_SPI_FLUSH_RX            0xE2 //�������� FIFO �� ����
#define NRF_SPI_REUSE_TX_PL         0xE3 //��������� ������������� ��������� ���������� ������
#define NRF_SPI_R_RX_PL_WID         0x60 //������ ������� ������ �� ����� FIFO
#define NRF_SPI_W_ACK_PAYLOAD       0xA8 //������ ������ ��� ������ ACK
#define NRF_SPI_W_TX_PAYLOAD_NO_ACK 0xB0 //������ ������ ��� �������� �� ��� ������������� ACK
#define NRF_SPI_NOP                 0xFF //������ �������

//����� ��������� ������� ���������
#define NRF_REGISTER_ADDRESS_MASK   0x1F

//����� ��������� ������ (PIPE)
#define NRF_REGISTER_PIPE_MASK      0x07

//������������ ����� ������ ������������ ����� SPI
#define NRF_SPI_MAXIMUM_DATA_EXCHANGE_SIZE 0x21 //1 ���� ������� + 32 ����� ������

//������������ ����� ������ ��� �������� ����� ����������
#define NRF_RF_MAXIMUM_DATA_EXCHANGE_SIZE  0x20 //32 ����� ������

//������ (�������) ���� ����������� �� ������ �� SPI �������� ������� �������
#define NRF_SPI_STATUS_DATA_BYTE_NUMBER    0x00

//���� ������
#define NRF_ERROR_CODE_SUCCESS              0x00 //��� ������
#define NRF_ERROR_CODE_INVALID_DATA_PARAMS  0x01 //��������� ���������
#define NRF_ERROR_CODE_SPI_EXCHANGE_ERROR   0x02 //������ ������ �� SPI
#define NRF_ERROR_CODE_TIMEOUT              0x03 //������ �������
#define NRF_ERROR_CODE_SEND_DATA_SUCCESS    0x04 //����� �������� ������
#define NRF_ERROR_CODE_RECEIVE_DATA_SUCCESS 0x05 //����� ����� ������
#define NRF_ERROR_CODE_CHANNEL_BUSY         0x06 //����������/��� �����
#define NRF_ERROR_CODE_SEND_MAX_RT_ERROR    0x07 //������ ���������� ������������� ����� ������� �������� ������

//�������� ������ ����� ����������
#define NRF_DATA_RATE_250K          0x00 //�������� ������ 250K
#define NRF_DATA_RATE_1M            0x01 //�������� ������ 1000K = 1M
#define NRF_DATA_RATE_2M            0x02 //�������� ������ 2000K = 2M

//�������� ��������� ������
#define NRF_DATA_RATE_250K_VALUE    250000  //�������� ������ 250K
#define NRF_DATA_RATE_1M_VALUE      1000000 //�������� ������ 1000K
#define NRF_DATA_RATE_2M_VALUE      2000000 //�������� ������ 2000K

//�������� ��������� �����������
#define NRF_RF_PWR_MINUS_18DBM      0x00 //�������� ����������� -18 ���
#define NRF_RF_PWR_MINUS_12DBM      0x01 //�������� ����������� -12 ���
#define NRF_RF_PWR_MINUS_6DBM       0x02 //�������� ����������� -6 ���
#define NRF_RF_PWR_0DBM             0x03 //�������� ����������� 0 ���

//�������� ��� � �������� ��� ������� ��������
#define NRF_RF_PWR_BIT_LEFT_SHIFT   0x01 //�� ���� ��� �����

//����� CRC
#define NRF_CRC_LENGTH_1_BYTE       0x00 //����� CRC 1 ����
#define NRF_CRC_LENGTH_2_BYTE       0x01 //����� CRC 2 �����

//����� ������ �� �������� � ����, ��� ������ � �������
#define NRF_ADDRESS_LENGTH_3_BYTE   0x00 //����� ������ 3 �����
#define NRF_ADDRESS_LENGTH_4_BYTE   0x01 //����� ������ 4 �����
#define NRF_ADDRESS_LENGTH_5_BYTE   0x02 //����� ������ 5 �����

//����� ������ �� �������� � ����, ��������
#define NRF_ADDRESS_LENGTH_3_BYTE_VALUE  0x03 //����� ������ 3 �����
#define NRF_ADDRESS_LENGTH_4_BYTE_VALUE  0x04 //����� ������ 4 �����
#define NRF_ADDRESS_LENGTH_5_BYTE_VALUE  0x05 //����� ������ 5 �����

//����������� � ������������ �������� ��� �������� ����� ��������� �������� ������, ��� � 1 - ��� 250 ���, Auto Retransmit Delay
#define NRF_ARD_MINIMUM             0x00 //250 ���
#define NRF_ARD_MAXIMUM             0x0F //4000 ���

//�������� ���� �������� ����� ��������� ���������
#define NRF_ARD_STEP_VALUE          250 //250 ���

//�������� ��� � ��������, ��� �������� ����� ��������� ���������
#define NRF_ARD_BIT_LEFT_SHIFT     0x04 //�� 4 ���� �����

//����������� � ������������ �������� ��� ����� ��������� ������� �������
#define NRF_ARC_MINIMUM             0x00 //0 - ��� ��������� �������
#define NRF_ARC_MAXIMUM             0x0F //15 - ��������

//�������� �� ��������� ��� �������� ����� ��������� ��������� ������� � ����� ��������� �������� �������
#define NRF_ARD_DEFAULT             0x00 //0 - 250 ���
#define NRF_ARC_DEFAULT             0x03 //3 - ��� ��������

//����������� � ������������ �������� ������ �����������, � ����� 1 ��� ������� � 2400 ���
#define NRF_RADIO_CHANNEL_NUMBER_MINIMUM 0x00 //����� 0
#define NRF_RADIO_CHANNEL_NUMBER_MAXIMUM 0x7F //����� 127
#define NRF_RADIO_CHANNEL_DEFAULT        0x02 //����� ������ �� ���������

//������ ����������� � ��������� �������
#define NRF_RADIO_CHANNEL_WIDTH 1    //1 ���
#define NRF_RADIO_CHANNEL_START 2400 //2400 ���

//����� ������� �� ���� � ������ �������
#define NRF_RX_DATA_PIPE_COUNT    6 //6 ������� �� ����
#define NRF_RX_DATA_PIPE_NUMBER_0 0 //����� 0
#define NRF_RX_DATA_PIPE_NUMBER_1 1 //����� 1
#define NRF_RX_DATA_PIPE_NUMBER_2 2 //����� 2
#define NRF_RX_DATA_PIPE_NUMBER_3 3 //����� 3
#define NRF_RX_DATA_PIPE_NUMBER_4 4 //����� 4
#define NRF_RX_DATA_PIPE_NUMBER_5 5 //����� 5

//�������� ��� � �������� ������� ��� ������ ������
#define NRF_RX_DATA_PIPE_NUMBER_LEFT_SHIFT 1 //�� 1 ��� �����

//�������� ��� ��� ������� FIFO �� ���� � �������� �������
#define NRF_RX_DATA_PIPE_RF_FIFO_EMPTY 0x0E

//�������� ��� ��� ��������� ����� ��������� �������
#define NRF_COUTN_LOST_PACKET_BIT_LEFT_SHIFT 0x04 //�������� �� 4 ���� �����

//������� ��������/����� ��� �������� CONIG
#define NRF_REGISTER_CONFIG_MASK_RX_DR  0x40 //��� ������� ���������� �� ����, 1 - ���������, 0 - ���������
#define NRF_REGISTER_CONFIG_MASK_TX_DS  0x20 //��� ������� ���������� �� ��������, 1 - ���������, 0 - ���������
#define NRF_REGISTER_CONFIG_MASK_MAX_RT 0x10 //��� ������� ���������� �� ������������ ����� ������������, 1 - ���������, 0 - ���������
#define NRF_REGISTER_CONFIG_EN_CRC      0x08 //��� ���������� CRC, 1- ���������
#define NRF_REGISTER_CONFIG_CRCO        0x04 //��� ����� CRC, 0 - 1 ����, 1 - 2 �����
#define NRF_REGISTER_CONFIG_PWR_UP      0x02 //��� ���������, 1 - POWER UP, 0 - POWER DOWN
#define NRF_REGISTER_CONFIG_PRIM_RX     0x01 //��� ���������� ������� ����/��������, 1 - ����, 0 - ��������

//������� ��������/����� ��� �������� EN_AA
#define NRF_REGISTER_EN_AA_ENAA_P5      0x20 //��� ���������� ������������� ������ ��� ������ 5
#define NRF_REGISTER_EN_AA_ENAA_P4      0x10 //��� ���������� ������������� ������ ��� ������ 4
#define NRF_REGISTER_EN_AA_ENAA_P3      0x08 //��� ���������� ������������� ������ ��� ������ 3
#define NRF_REGISTER_EN_AA_ENAA_P2      0x04 //��� ���������� ������������� ������ ��� ������ 2
#define NRF_REGISTER_EN_AA_ENAA_P1      0x02 //��� ���������� ������������� ������ ��� ������ 1
#define NRF_REGISTER_EN_AA_ENAA_P0      0x01 //��� ���������� ������������� ������ ��� ������ 0

//������� ��������/����� ��� �������� EN_RXADDR
#define NRF_REGISTER_EN_RXADDR_ERX_5    0x20 //��� ���������� �������� ������ 5
#define NRF_REGISTER_EN_RXADDR_ERX_4    0x10 //��� ���������� �������� ������ 4
#define NRF_REGISTER_EN_RXADDR_ERX_3    0x08 //��� ���������� �������� ������ 3
#define NRF_REGISTER_EN_RXADDR_ERX_2    0x04 //��� ���������� �������� ������ 2
#define NRF_REGISTER_EN_RXADDR_ERX_1    0x02 //��� ���������� �������� ������ 1
#define NRF_REGISTER_EN_RXADDR_ERX_0    0x01 //��� ���������� �������� ������ 0

//������� ��������/����� ��� �������� SETUP_AW
#define NRF_REGISTER_SETUP_AW_3_BYTES   0x01 //����� ������ 3 �����
#define NRF_REGISTER_SETUP_AW_4_BYTES   0x02 //����� ������ 4 �����
#define NRF_REGISTER_SETUP_AW_5_BYTES   0x03 //����� ������ 5 ����

//������� ��������/����� ��� �������� SETUP_RETR
#define NRF_REGISTER_SETUP_RETR_ARD_MASK 0xF0 //����� ��� ��� �������� ����� ��������� ��������� ������, Auto Retransmit Delay, ��� 250 ���
#define NRF_REGISTER_SETUP_RETR_ARC_MASK 0x0F //����� ��� ����� ��������� ������� �������

//������� ��������/����� ��� �������� SETUP_RETR
#define NRF_REGISTER_RF_CH_MASK          0x7F //����� ��� ������� �����������

//������� ��������/����� ��� �������� RF_SETUP
#define NRF_REGISTER_RF_SETUP_CONT_WAVE   0x80 //��� ��������� ������������ ������� ������� �� ������
#define NRF_REGISTER_RF_SETUP_RF_DR_LOW   0x20 //��� �������� Data Rate � 250�, ��� ������� ��������
#define NRF_REGISTER_RF_SETUP_PLL_LOCK    0x10 //��� PLL Lock ��� ������������
#define NRF_REGISTER_RF_SETUP_RF_DR_HIGH  0x08 //��� �������� Data Rate, ��� ������� ��������
#define NRF_REGISTER_RF_SETUP_RF_PWR_MASK 0x06 //����� ��� ��� ������� �������� �����������

//������� ��������/����� ��� �������� STATUS
#define NRF_REGISTER_STATUS_RX_DR_BIT     0x40 //��� ���������� Data Ready RX FIFO interrupt
#define NRF_REGISTER_STATUS_TX_DS_BIT     0x20 //��� ���������� Data Sent TX FIFO interrupt
#define NRF_REGISTER_STATUS_MAX_RT_BIT    0x10 //��� ���������� Maximum number of TX retransmits interrupt
#define NRF_REGISTER_STATUS_RX_P_NO_MASK  0x0E //����� ��� ��� ������ ������ ���������� ������ � FIFO
#define NRF_REGISTER_STATUS_TX_FULL_BIT   0x01 //��� ������������ FIFO �� ��������

//������� ��������/����� ��� �������� OBSERVE_TX
#define NRF_REGISTER_OBSERVE_TX_PLOS_CNT_MASK 0xF0 //����� ��� ����� ��������� �������
#define NRF_REGISTER_OBSERVE_TX_ARC_CNT_MASK  0x0F //����� ��� ����� ���������� �������� �������

//������� ��������/����� ��� �������� RPD
#define NRF_REGISTER_RPD_RX_POWER_DETECTOR_BIT 0x01 //��� ���������� ������ ��������� ��������, ����� -64 ���

//������� ��������/����� ��� ��������� RX_PW_P0 - RX_PW_P5
#define NRF_REGISTER_RX_PW_P0_P5_MASK     0x3F//����� ��������� ����� �������� ���� ������� (PIPE)

//������� ��������/����� ��� �������� FIFO_STATUS
#define NRF_REGISTER_FIFO_STATUS_TX_REUSE_BIT 0x40 //��� ���������� ������������� ��������� ����������� �������
#define NRF_REGISTER_FIFO_STATUS_TX_FULL      0x20 //���� ������������ FIFO �� ��������
#define NRF_REGISTER_FIFO_STATUS_TX_EMPTY     0x10 //���� ������� FIFO �� ��������
#define NRF_REGISTER_FIFO_STATUS_RX_FULL      0x02 //���� ������������ FIFO �� ����
#define NRF_REGISTER_FIFO_STATUS_RX_EMPTY     0x01 //���� ������� FIFO �� ����

//������� ��������/����� ��� �������� DYNPD
#define NRF_REGISTER_DYNPD_DPL_P5   0x20 //��� ���������� ������������ ����� ������� ��� ������ (PIPE) 5
#define NRF_REGISTER_DYNPD_DPL_P4   0x10 //��� ���������� ������������ ����� ������� ��� ������ (PIPE) 4
#define NRF_REGISTER_DYNPD_DPL_P3   0x08 //��� ���������� ������������ ����� ������� ��� ������ (PIPE) 3
#define NRF_REGISTER_DYNPD_DPL_P2   0x04 //��� ���������� ������������ ����� ������� ��� ������ (PIPE) 2
#define NRF_REGISTER_DYNPD_DPL_P1   0x02 //��� ���������� ������������ ����� ������� ��� ������ (PIPE) 1
#define NRF_REGISTER_DYNPD_DPL_P0   0x01 //��� ���������� ������������ ����� ������� ��� ������ (PIPE) 0

//������� ��������/����� ��� �������� FEATURE
#define NRF_REGISTER_FEATURE_EN_DPL     0x04 //��� ���������� ������������ ����� �������
#define NRF_REGISTER_FEATURE_EN_ACK_PAY 0x02 //��� ���������� �������� ������ � �������������� ACK
#define NRF_REGISTER_FEATURE_EN_DYN_ACK 0x01 //��� ���������� ������� NRF_SPI_W_TX_PAYLOAD_NO_AC

//����� ��������/��������� � �� ��������
#define NRF_TIMERS_COUNT  0x02 //��� �������
#define NRF_TIMER_DELAY   0x00 //��� ����
#define NRF_TIMER_TIMEOUT 0x01 //��� ���������

//�������� ���� � ���������
#define NRF_TIME_POWER_UP          0x03 //3 �� � �������, �������� 1.5 ��
#define NRF_TIME_CE_START_TRANSMIT 0x0E //15 ��� � �������, ���������� 10 ���
#define NRF_TIME_CE_START_RECEIVE  0x0E //����� ����� ������

//��������� �������� ������������ ����� ����� ����������
#define NRF_MACHINE_STATE_FREE          0x00 //������� ��������� ��������
#define NRF_MACHINE_STATE_START_SEND    0x01 //��������� ������ ��������
#define NRF_MACHINE_STATE_WAIT_SEND     0x02 //��������� �������� ���������� ��������
#define NRF_MACHINE_STATE_START_RECEIVE 0x03 //��������� ������ �����
#define NRF_MACHINE_STATE_WAIT_RECEIVE  0x04 //��������� �������� ���������� �����

#endif /* NRF24L01PLUS_DEFINES_H_ */
