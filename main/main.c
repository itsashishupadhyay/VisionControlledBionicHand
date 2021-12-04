#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "esp_spi_flash.h"
#include "Drivers.h"
#include "AWS_MQTT.h"

TaskHandle_t TaskI2C;
TaskHandle_t TaskSendCommand = NULL;

void app_main(void)
{
    aws_sub_pub_app_main();
    ChipInfo();
    xTaskCreatePinnedToCore(
        I2cTask,   /* Function to implement the task */
        "I2cTask", /* Name of the task */
        2 * 1024,  /* Stack size in words */
        NULL,      /* Task input parameter */
        0,         /* Priority of the task */
        &TaskI2C,  /* Task handle. */
        0);        /* Core where the task should run */

    // xTaskCreatePinnedToCore(
    //     SendCommandTask,   /* Function to implement the task */
    //     "AppendQueueTask", /* Name of the task */
    //     2 * 1024,          /* Stack size in words */
    //     NULL,              /* Task input parameter */
    //     0,                 /* Priority of the task */
    //     &TaskSendCommand,  /* Task handle. */
    //     0);                /* Core where the task should run */
}