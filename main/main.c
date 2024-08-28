#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "string.h"
#include "driver/adc.h"
#include "driver/gpio.h"

// Định nghĩa các thông số cho UART

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE 1024
void init(void);    // Hàm cấu hình UART

static void tx_task(void *arg);

void tx_task(void *arg); // send data
void configADC(void);

int encodePIRVols(
    uint8_t index,
    uint16_t pir_vol0,
    uint16_t pir_vol1,
    uint16_t pir_vol2,
    uint16_t pir_vol3,
    uint16_t pir_vol4,
    uint8_t *encodedData);

void app_main(void)
{
    init(); // Khởi tạo các cấu hình cho UART

    xTaskCreatePinnedToCore(tx_task, "tx_task", 8196, NULL, 1, NULL, 0); // Chạy task adc_task ở core 0
};
void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
};
void configADC(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12); // Set ADC resolution to 12 bits

    // Configure ADC1 channels for full range with 11 dB attenuation
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11); // GPIO 39, max input ~3.9V
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11); // GPIO 32, max input ~3.9V
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11); // GPIO 33, max input ~3.9V
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // GPIO 34, max input ~3.9V
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11); // GPIO 35, max input ~3.9V
};

int sendData(char data[])
{
    // const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, 12);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
};
void tx_task(void *arg)
{

    while (1)
    {
        configADC();

        char package[12];
        for (uint8_t index = 0; index < 100; index++)
        {
            uint16_t raw_data0 = adc1_get_raw(ADC1_CHANNEL_3);
            uint16_t raw_data1 = adc1_get_raw(ADC1_CHANNEL_4);
            uint16_t raw_data2 = adc1_get_raw(ADC1_CHANNEL_5);
            uint16_t raw_data3 = adc1_get_raw(ADC1_CHANNEL_6);
            uint16_t raw_data4 = adc1_get_raw(ADC1_CHANNEL_7);

            int length = encodePIRVols(index, raw_data0, raw_data1, raw_data2, raw_data3, raw_data4, (uint8_t*)package);
            printf("encoded bytes: %d\ncontent: %s", length, package);
           
            sendData(package);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
};

int encodePIRVols(
    uint8_t index,
    uint16_t pir_vol0,
    uint16_t pir_vol1,
    uint16_t pir_vol2,
    uint16_t pir_vol3,
    uint16_t pir_vol4,
    uint8_t *encodedData)
{
    int sum = 0;
    // index
    encodedData[0] = index;

    // vol0
    encodedData[1] = (uint8_t)(pir_vol0 >> 8);
    encodedData[2] = (uint8_t)(pir_vol0);

    // vol1
    encodedData[3] = (uint8_t)(pir_vol1 >> 8);
    encodedData[4] = (uint8_t)(pir_vol1);

    // vol2
    encodedData[5] = (uint8_t)(pir_vol2 >> 8);
    encodedData[6] = (uint8_t)(pir_vol2);

    // vol3
    encodedData[7] = (uint8_t)(pir_vol3 >> 8);
    encodedData[8] = (uint8_t)(pir_vol3);

    // vol4
    encodedData[9] = (uint8_t)(pir_vol4 >> 8);
    encodedData[10] = (uint8_t)(pir_vol4);

    for (int i = 0; i <= 10; i++)
        sum += encodedData[i];

    encodedData[11] = (uint8_t)(sum);

    return 12;
};