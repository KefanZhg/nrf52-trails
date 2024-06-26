#ifndef CONFIG_H
#define CONFIG_H

// File includes
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "nordic_common.h"

#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_nus.h"
#include "bsp_btn_ble.h"

#include "nrf.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"

#include "nrf_ble_gatt.h"

#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"
#include "nrf_drv_spi.h"

#include "nrfx_twi.h"

#include "boards.h"

#include "app_error.h"
#include "app_timer.h"
#include "app_uart.h"
#include "app_util.h"
#include "app_util_platform.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

/* FreeRTOS related */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Include drivers
#include "i2c.h"
#include "eeprom.h"
#include "tmag5170_nrf5.h"

// Pin definitions
#define LED_PIN 46
#define BTN_PIN 8

#define TWI_SDA_PIN  17
#define TWI_SCL_PIN  13

#define SPI_SCK_PIN  38
#define SPI_MOSI_PIN 36
#define SPI_MISO_PIN 34

#define TMAG5170_NUM_SENSORS 5
#define TMAG5170_CS_PINS {32, 22, 24, 9, 10}

#define JOYSTICK_SW_PIN 16
#define JOYSTICK_X_AIN   6
#define JOYSTICK_Y_AIN   4

#define APP_TWI_INSTANCE_ID 0

#define EEPROM_ADDRESS (0x50) // The I2C address of the AT24C256, might need adjustment based on the hardware setup

// External variables
extern i2c_t app_i2c;
extern const nrf_drv_twi_t app_twi;

extern eeprom_t eeprom;

extern tmag5170_t tmag5170s[TMAG5170_NUM_SENSORS];

#endif // CONFIG_H