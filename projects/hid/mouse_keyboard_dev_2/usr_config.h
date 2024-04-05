#ifndef USR_CONFIG_H_
#define USR_CONFIG_H_

#include "nrf.h"

#include "nrf_cli.h"
#include "nrf_log.h"
#include "nrf_delay.h"

#include "nrfx_saadc.h"

#include "nrf_drv_saadc.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_button.h"
#include "app_timer.h"
#include "app_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd_hid_mouse.h"
#include "app_usbd_hid_kbd.h"
#include "app_usbd_dummy.h"
#include "app_error.h"

#include "boards.h"
#include "inttypes.h"

#define USR_JOYSTICK_BTN_PIN    16

#define USR_MOUSE_X_ADC_INPUT NRF_SAADC_INPUT_AIN4
#define USR_MOUSE_Y_ADC_INPUT NRF_SAADC_INPUT_AIN6
#define USR_MOUSE_ADC_SAMPLE_NUM 8
#define USR_MOUSE_ADC_OFFSET     430
#define USR_MOUSE_SIGNAL_MARGIN  10
#define USR_MOUSE_SIGNAL_SCALE   40
#define USR_MOUSE_TIMER_INTERVAL 10

#endif /* USR_CONFIG_H_ */
