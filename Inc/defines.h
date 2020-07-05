#ifndef __DEFINES_H
#define __DEFINES_H
 
//#define GPS_USART	USART1
#define GPS_USART_PINSPACK		TM_USART_PinsPack_1

//#define USARTx	USART1
//#define USART_FLAG_TXE                       ((uint16_t)0x0080)

////Disable GPGGA statement
//#define GPS_DISABLE_GPGGA
////Disable GPRMC statement
//#define GPS_DISABLE_GPRMC
////Disable GPGSA statement
//#define GPS_DISABLE_GPGSA
////Disable GPGSV statement
//#define GPS_DISABLE_GPGSV

////Change X with possible U(S)ARTs: USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8, for STM32F0xx additions: USART4, USART5, USART7, USART8
////Set flow control
//#define TM_USARTx_HARDWARE_FLOW_CONTROL    TM_USART_HardwareFlowControl_None
////Set mode
//#define TM_USART1_MODE                     USART_MODE_TX_RX
////Set parity
//#define TM_USARTx_PARITY                   USART_PARITY_NONE
////Set stopbits
//#define TM_USARTx_STOP_BITS                USART_STOPBITS_1
////Set USART datasize
//#define TM_USARTx_WORD_LENGTH              UART_WORDLENGTH_8B

 
#endif


