#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <dht.h>
#include "driver/gpio.h"

#include "sensor_driver.h"

void get_temperature_and_humidity()
{
    dht_read_float_data(DHT_TYPE_AM2301, (gpio_num_t)DHT22_PIN, &humidity, &temperature);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
}