#include "Drivers.h"
#include "sdkconfig.h"

#define I2C_MASTER_SDA_IO (int)23
#define I2C_MASTER_SCL_IO (int)22
#define OscillatorFrequency 27000000
#define I2C_MASTER_NUM I2C_NUM_1
#define I2C_MASTER_FREQ_HZ 100000

#define ESP_SLAVE_ADDR 0x40 /*!< ESP32 slave address, you can set any 7bit value */

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

#define MINServoAnalog 409
#define MAXServoAnalog 1693

static const char *TAG = "Drivers";
QueueHandle_t xFingerPowerQueue = NULL;

void ChipInfo(void)
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
}

static esp_err_t i2c_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK)
    {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ____________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | register + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------|----------------------|------|
 *
 */
static esp_err_t __attribute__((unused)) i2c_master_write_slave(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, (i2c_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // write the data
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief test code to read i2c slave device with registered interface
 * _______________________________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | register + ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------|----------------------|--------------------|------|
 *
 */
static esp_err_t __attribute__((unused)) i2c_master_read_slave(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t *data_rd, size_t size)
{
    if (size == 0)
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, (i2c_addr << 1), ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // Send repeated start
    i2c_master_start(cmd);
    // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, (i2c_addr << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1)
    {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int PCA9685(void)
{
    int ret;
    uint8_t regMODE1 = 0x00;
    uint8_t regMODE2 = 0x01;
    uint8_t Data_MODE1 = 0x21;
    uint8_t Data_MODE2 = 0x04;
    ret = i2c_master_write_slave(I2C_MASTER_NUM, ESP_SLAVE_ADDR, regMODE1, &Data_MODE1, sizeof(uint8_t));
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Mode1 Error %s:", esp_err_to_name(ret));
    }
    ret = i2c_master_write_slave(I2C_MASTER_NUM, ESP_SLAVE_ADDR, regMODE2, &Data_MODE2, sizeof(uint8_t));
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Mode1 Error %s:", esp_err_to_name(ret));
    }
    return ret;
}

void I2cTask(void *parameter)
{
    int ret;
    ret = i2c_init();
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Driver install %s: ", esp_err_to_name(ret));
    }
    else
    {
        printf("I2C driver Install Success\r\n");
    }

    if (PCA9685() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initilized PCA9685");
        while (1)
            ;
    }

    xFingerPowerQueue = xQueueCreate(10, sizeof(FingerActivation));

    if (xFingerPowerQueue == NULL)
    {
        printf("The Queue Creation Failed");
    }

    // vTaskResume(TaskSendCommand);

    while (1)
    {
        ActuationTask();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void SendCommandTask(void *parameter)
{
    vTaskSuspend(NULL);
    FingerActivation SendTaskQueue;
    ESP_LOGI(TAG, "Send Command Task Enabled");

    while (true)
    {

        for (uint8_t i = 0; i < 5; i++)
        {
            for (uint8_t j = 0; j <= 100; j = j + 10)
            {
                SendTaskQueue.Finger = i;
                SendTaskQueue.PowerPercent = j;
            }
            xQueueSend(xFingerPowerQueue, (void *)&SendTaskQueue, (TickType_t)0);
            vTaskDelay(1500 / portTICK_PERIOD_MS);
        }
        for (uint8_t i = 0; i < 5; i++)
        {
            for (int8_t j = 100; j >= 00; j = (j - 10))
            {
                SendTaskQueue.Finger = i;
                SendTaskQueue.PowerPercent = j;
            }
            xQueueSend(xFingerPowerQueue, (void *)&SendTaskQueue, (TickType_t)0);
            vTaskDelay(1500 / portTICK_PERIOD_MS);
        }
    }
}

void uint16toRegconverter(uint16_t Offtime, uint8_t *HighReg, uint8_t *LowReg)
{
    *LowReg = (uint8_t)(((uint16_t)Offtime >> 0) & 0xFF);
    *HighReg = (uint8_t)((uint16_t)Offtime >> 8) & 0xFF;
}

void ActuationTask(void)
{
    FingerActivation CurrentTaskQueue;
    uint8_t ret;
    uint8_t reg_SERVO_x__ON_L, reg_SERVO_x__ON_H, reg_SERVO_x__OFF_L, reg_SERVO_x__OFF_H;
    const uint8_t SERVO_x__ON_L = 0b00000001;
    const uint8_t SERVO_x__ON_H = 0x00;
    uint8_t SERVO_x__OFF_L, SERVO_x__OFF_H;
    uint8_t ReadAddr = 0xFE;
    uint8_t read_data = 0;

    if (xFingerPowerQueue != NULL)
    {
        if (xQueueReceive(xFingerPowerQueue,
                          &(CurrentTaskQueue),
                          (TickType_t)10) == pdPASS)
        {
            uint16_t Percent2Analog = (CurrentTaskQueue.Finger != Thumb) ? MINServoAnalog + ((CurrentTaskQueue.PowerPercent) * MAXServoAnalog / (100))
                                                                         : MINServoAnalog + ((100 - CurrentTaskQueue.PowerPercent) * MAXServoAnalog / (100));
            switch (CurrentTaskQueue.Finger)
            {
            case Thumb:
            {
                printf("THUMB\r\n");
                reg_SERVO_x__ON_L = 0x06;
                reg_SERVO_x__ON_H = 0x07;
                reg_SERVO_x__OFF_L = 0x08;
                reg_SERVO_x__OFF_H = 0x09;
                uint16toRegconverter(Percent2Analog, &SERVO_x__OFF_H, &SERVO_x__OFF_L);

                break;
            }
            case Index:
            {
                printf("INDEX\r\n");
                reg_SERVO_x__ON_L = 0x0A;
                reg_SERVO_x__ON_H = 0x0B;
                reg_SERVO_x__OFF_L = 0x0C;
                reg_SERVO_x__OFF_H = 0x0D;
                uint16toRegconverter(Percent2Analog, &SERVO_x__OFF_H, &SERVO_x__OFF_L);

                break;
            }
            case Middle:
            {
                printf("MIDDLE\r\n");
                reg_SERVO_x__ON_L = 0x0E;
                reg_SERVO_x__ON_H = 0x0F;
                reg_SERVO_x__OFF_L = 0x10;
                reg_SERVO_x__OFF_H = 0x11;
                uint16toRegconverter(Percent2Analog, &SERVO_x__OFF_H, &SERVO_x__OFF_L);

                break;
            }
            case Ring:
            {
                printf("RING\r\n");
                reg_SERVO_x__ON_L = 0x12;
                reg_SERVO_x__ON_H = 0x13;
                reg_SERVO_x__OFF_L = 0x14;
                reg_SERVO_x__OFF_H = 0x15;
                uint16toRegconverter(Percent2Analog, &SERVO_x__OFF_H, &SERVO_x__OFF_L);

                break;
            }
            case Pinkey:
            {
                printf("PINKEY\r\n");
                reg_SERVO_x__ON_L = 0x16;
                reg_SERVO_x__ON_H = 0x17;
                reg_SERVO_x__OFF_L = 0x18;
                reg_SERVO_x__OFF_H = 0x19;
                uint16toRegconverter(Percent2Analog, &SERVO_x__OFF_H, &SERVO_x__OFF_L);

                break;
            }
            case All:
            {
                printf("ALL\r\n");
                reg_SERVO_x__ON_L = 0xFA; //lower order 8 bit
                reg_SERVO_x__ON_H = 0xFB; //higher order 4bits
                reg_SERVO_x__OFF_L = 0xFC;
                reg_SERVO_x__OFF_H = 0xFD;
                // uint16toRegconverter(10, &SERVO_x__OFF_H, &SERVO_x__OFF_L);

                break;
            }
            default:
                printf("Should not reach here, with val Finger %d Percentage %d\r\n", CurrentTaskQueue.Finger, CurrentTaskQueue.PowerPercent);
                reg_SERVO_x__ON_L = 0;
                reg_SERVO_x__ON_H = 0;
                reg_SERVO_x__OFF_L = 0;
                reg_SERVO_x__OFF_H = 0;
            }
            printf("\r\n REG VAL ON %02x %02x OFF %02x %02x\r\n", SERVO_x__ON_H, SERVO_x__ON_L, SERVO_x__OFF_H, SERVO_x__OFF_L);
            ret = i2c_master_write_slave(I2C_MASTER_NUM, ESP_SLAVE_ADDR, reg_SERVO_x__ON_L, &SERVO_x__ON_L, sizeof(uint8_t));
            if (ret != ESP_OK)
            {
                ESP_LOGW(TAG, "Write ON_L Error %s:", esp_err_to_name(ret));
            }

            ret = i2c_master_write_slave(I2C_MASTER_NUM, ESP_SLAVE_ADDR, reg_SERVO_x__ON_H, &SERVO_x__ON_H, sizeof(uint8_t));
            if (ret != ESP_OK)
            {
                ESP_LOGW(TAG, "Write ON_H Error %s:", esp_err_to_name(ret));
            }

            ret = i2c_master_write_slave(I2C_MASTER_NUM, ESP_SLAVE_ADDR, reg_SERVO_x__OFF_L, &SERVO_x__OFF_L, sizeof(uint8_t));
            if (ret != ESP_OK)
            {
                ESP_LOGW(TAG, "Write OFF_L Error %s:", esp_err_to_name(ret));
            }

            ret = i2c_master_write_slave(I2C_MASTER_NUM, ESP_SLAVE_ADDR, reg_SERVO_x__OFF_H, &SERVO_x__OFF_H, sizeof(uint8_t));
            if (ret != ESP_OK)
            {
                ESP_LOGW(TAG, "Write OFF_H Error %s:", esp_err_to_name(ret));
            }

            ret = i2c_master_read_slave(I2C_MASTER_NUM, ESP_SLAVE_ADDR, ReadAddr, &read_data, sizeof(read_data));
            if (ret != ESP_OK)
            {
                ESP_LOGW(TAG, "Read Error %s: ", esp_err_to_name(ret));
            }
            else
            {
                printf("The reg Data is %02x!\r\n", read_data);
            }
        }
    }
}

void OpenFingers()
{
    FingerActivation SendTaskQueue;
    for (uint8_t i = 0; i < 5; i++)
    {
        SendTaskQueue.Finger = i;
        SendTaskQueue.PowerPercent = 100;

        xQueueSend(xPushMessageQueue, (void *)&SendTaskQueue, (TickType_t)0);
        xQueueSend(xFingerPowerQueue, (void *)&SendTaskQueue, (TickType_t)0);
        // vTaskDelay(1500 / portTICK_PERIOD_MS);
    }
}
void CloseFingers()
{
    FingerActivation SendTaskQueue;
    for (uint8_t i = 0; i < 5; i++)
    {
        SendTaskQueue.Finger = i;
        SendTaskQueue.PowerPercent = 0;

        xQueueSend(xPushMessageQueue, (void *)&SendTaskQueue, (TickType_t)0);
        xQueueSend(xFingerPowerQueue, (void *)&SendTaskQueue, (TickType_t)0);
        // vTaskDelay(1500 / portTICK_PERIOD_MS);
    }
}