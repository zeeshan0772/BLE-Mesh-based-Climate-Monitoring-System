#ifndef _SENSOR_DRIVER_H
#define _SENSOR_DRIVER_H

#define DHT22_PIN 17

float temperature;
float humidity;

void get_temperature_and_humidity();

#endif