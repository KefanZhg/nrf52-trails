#include "app.h"


void app_init(void)
{
    ret_code_t err_code;
    UNUSED_VARIABLE(err_code);

    // Init peripherals
    // const nrf_drv_twi_config_t twi_config = {
    //    .scl                = TWI_SCL_PIN,
    //    .sda                = TWI_SDA_PIN,
    //    .frequency          = NRF_DRV_TWI_FREQ_400K,
    //    .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
    //    .clear_bus_init     = false
    // };

    // err_code = nrf_drv_twi_init(&app_twi, &twi_config, NULL, NULL);
    // APP_ERROR_CHECK(err_code);

    // nrf_drv_twi_enable(&app_twi);
    i2c_init(&app_i2c, TWI_SDA_PIN, TWI_SCL_PIN, NRF_DRV_TWI_FREQ_100K);


    // Init modules


    // Init threads
    if (pdPASS != xTaskCreate(app_thread,
                              "APP",
                              APP_TASK_STACK_SIZE,
                              NULL,
                              APP_TASK_PRIORITY,
                              &app_task_handle))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    // Init timers

}


// Application thread
// Only for testing purposes
TaskHandle_t app_task_handle;
void app_thread(void * arg)
{
    static uint32_t count = 0;
    static uint32_t read_count = 0;
    UNUSED_VARIABLE(count);
    UNUSED_VARIABLE(read_count);
    UNUSED_PARAMETER(arg);
    for(;;)
    {
        // Sleep 1 second
        vTaskDelay(pdMS_TO_TICKS(900));
        // Print time
        // NRF_LOG_DEBUG("Time: %d", xTaskGetTickCount());
        count += 4;
        // Write to EEPROM
        eeprom_write(count & 0x0FFF, (uint8_t *)&count, sizeof(count));
        // // Read from EEPROM
        vTaskDelay(pdMS_TO_TICKS(10));
        eeprom_read(count & 0x0FFF, (uint8_t *)&read_count, sizeof(read_count));
        // // Print read value
        i2c_lock(&app_i2c);
        NRF_LOG_DEBUG("Write: 0x%08X, Read: 0x%08X", count, read_count);
        i2c_unlock(&app_i2c);
    }
}
