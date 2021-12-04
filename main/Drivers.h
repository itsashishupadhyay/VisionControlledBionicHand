#ifndef __DRIVER__IIC__
#define __DRIVER__IIC__


#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "AWS_MQTT.h"
extern QueueHandle_t xFingerPowerQueue;
typedef enum{
    Thumb = 0,
    Index,
    Middle,
    Ring,
    Pinkey,
    All
}fingers;

typedef struct{
    fingers Finger;
    uint8_t PowerPercent;
}FingerActivation;
extern TaskHandle_t TaskSendCommand;
void ChipInfo(void);
void I2cTask(void * parameter);
void SendCommandTask(void *parameter);
void ActuationTask( void );
static esp_err_t  __attribute__((unused))  i2c_master_read_slave(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size);
static esp_err_t __attribute__((unused))  i2c_master_write_slave(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_wr, size_t size);
void uint16toRegconverter(uint16_t Offtime, uint8_t *HighReg, uint8_t *LowReg);
void OpenFingers(void);
void CloseFingers(void);
// static esp_err_t i2c_init(void);


#endif