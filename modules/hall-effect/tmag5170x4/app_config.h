#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "stdint.h"

#define NRF_LOG_BACKEND_UART_TX_PIN 26
#define NRF_LOG_BACKEND_UART_RX_PIN 6

#define LED_PIN 46
#define BTN_PIN 8

#define SPI_SCK_PIN  38
#define SPI_MOSI_PIN 36
#define SPI_MISO_PIN 34

#define TMAG5170_NUM_SENSORS 4

#define TMAG5170_CS_PIN_0 32
#define TMAG5170_CS_PIN_1 24
#define TMAG5170_CS_PIN_2 10
#define TMAG5170_CS_PIN_3 29

#define TMAG5170_ALERT_PIN_0 22
#define TMAG5170_ALERT_PIN_1 9
#define TMAG5170_ALERT_PIN_2 45
#define TMAG5170_ALERT_PIN_3 21

#define TMAG5170_HI_THRESH X_THRX_CONFIG_X_HI_THRESHOLD_5FS
#define TMAG5170_LO_THRESH X_THRX_CONFIG_X_LO_THRESHOLD_5FS

#define HIGH 1
#define LOW  0

#define delay_us(n) nrf_delay_us(n)
#define delay_ms(n) nrf_delay_ms(n)

// #define setCS(n)             nrf_gpio_pin_write(0, n)
// #define setALERT(n)          nrf_gpio_pin_write(0, n)

// <323584=> 1200 baud 
// <643072=> 2400 baud 
// <1290240=> 4800 baud 
// <2576384=> 9600 baud 
// <3862528=> 14400 baud 
// <5152768=> 19200 baud 
// <7716864=> 28800 baud 
// <10289152=> 38400 baud 
// <15400960=> 57600 baud 
// <20615168=> 76800 baud 
// <30801920=> 115200 baud 
// <61865984=> 230400 baud 
// <67108864=> 250000 baud 
// <121634816=> 460800 baud 
// <251658240=> 921600 baud 
// <268435456=> 1000000 baud 
#define APP_CONFIG_DEFAULT_UART_BAUDRATE 268435456

#define NRFX_UARTE_DEFAULT_CONFIG_BAUDRATE APP_CONFIG_DEFAULT_UART_BAUDRATE
#define NRFX_UART_DEFAULT_CONFIG_BAUDRATE APP_CONFIG_DEFAULT_UART_BAUDRATE
#define UART_DEFAULT_CONFIG_BAUDRATE APP_CONFIG_DEFAULT_UART_BAUDRATE
#define NRF_LOG_BACKEND_UART_BAUDRATE APP_CONFIG_DEFAULT_UART_BAUDRATE


// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 
#define NRF_LOG_DEFAULT_LEVEL 4


#endif // APP_CONFIG_H
