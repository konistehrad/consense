#define LOGNAME "SISWI"

#include <hk.h>
#include <hk_fascade.h>

#include <string.h>
#include <bmp280.h>
#include <esp_pm.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>

typedef float HkDecimal;

const gpio_num_t SDA_GPIO = GPIO_NUM_21;
const gpio_num_t SCL_GPIO = GPIO_NUM_22;

HkDecimal pressure = 0;
HkDecimal temperature = 0;
HkDecimal humidity = 0;

void* chr_temperature_ptr = NULL;
void* chr_humidity_ptr = NULL;

esp_err_t bmp280_read_double(bmp280_t *dev, double *temperature, double *pressure, double *humidity)
{
    int32_t fixed_temperature;
    uint32_t fixed_pressure;
    uint32_t fixed_humidity;
    ESP_ERROR_CHECK(bmp280_read_fixed(dev, &fixed_temperature, &fixed_pressure, humidity ? &fixed_humidity : NULL));
    *temperature = (double)fixed_temperature / 100;
    *pressure = (double)fixed_pressure / 256;
    if (humidity)
        *humidity = (double)fixed_humidity / 1024;

    return ESP_OK;
}

void on_identify()
{
    ESP_LOGI(LOGNAME, "Identify");
}

esp_err_t on_read_temp(hk_mem *response)
{
    hk_mem_append_buffer(response, (char *)&temperature, sizeof(HkDecimal));
    return ESP_OK;
}

esp_err_t on_read_humid(hk_mem *response)
{
    hk_mem_append_buffer(response, (char *)&humidity, sizeof(HkDecimal));
    return ESP_OK;
}

void bmp280_loop(void *pvParameters)
{
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_1, 1, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));

    bool bme280p = dev.id == BME280_CHIP_ID;
    ESP_LOGI("BMP280", "Found %s\n", bme280p ? "BME280" : "BMP280");

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(250));
        float fTemp, fPres, fHum;
        if (bmp280_read_float(&dev, &fTemp, &fPres, &fHum) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }
        pressure = (HkDecimal)fPres;
        temperature = (HkDecimal)fTemp;
        humidity = (HkDecimal)fHum;
        ESP_LOGI("BMP280", "T:%.2fC H:%.2f P:%2.f\n", fTemp, fHum, fPres);
        if(chr_humidity_ptr != NULL) hk_notify(chr_humidity_ptr);
        if(chr_temperature_ptr != NULL) hk_notify(chr_temperature_ptr);
        vTaskDelay(pdMS_TO_TICKS(9500));
    }
}

void hk_setup(void)
{
    hk_setup_start();
    hk_setup_add_accessory("Enviroment Sensor", "Conrad's Closet", "ESP32 D1 Mini", "0000001", "1", on_identify);
    hk_setup_add_srv(HK_SRV_TEMPERATURE_SENSOR, true, false);
    hk_setup_add_chr(HK_CHR_CURRENT_TEMPERATURE, on_read_temp, NULL, true, &chr_temperature_ptr); 
    
    hk_setup_add_srv(HK_SRV_HUMIDITY_SENSOR, true, false);
    hk_setup_add_chr(HK_CHR_CURRENT_RELATIVE_HUMIDITY, on_read_humid, NULL, true, &chr_humidity_ptr); 
    hk_setup_finish();

    hk_init("ConSense", HK_CAT_SENSOR, "026-62-932");
    ESP_LOGI(LOGNAME, "HK initialization complete.\n");
}

void pm_init(void)
{
#if CONFIG_PM_ENABLE
    // Configure dynamic frequency scaling:
    // maximum and minimum frequencies are set in sdkconfig,
    // automatic light sleep is enabled if tickless idle support is enabled.
#if CONFIG_IDF_TARGET_ESP32
    esp_pm_config_esp32_t pm_config = {
#elif CONFIG_IDF_TARGET_ESP32S2
    esp_pm_config_esp32s2_t pm_config = {
#elif CONFIG_IDF_TARGET_ESP32C3
    esp_pm_config_esp32c3_t pm_config = {
#elif CONFIG_IDF_TARGET_ESP32S3
    esp_pm_config_esp32s3_t pm_config = {
#endif
            .max_freq_mhz = 160,
            .min_freq_mhz = 80,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
            .light_sleep_enable = true
#endif
    };

    ESP_ERROR_CHECK( esp_pm_configure(&pm_config) );
#endif
}

void app_main(void)
{
    ESP_LOGI(LOGNAME, "SDK version:%s\n", esp_get_idf_version());
    ESP_LOGI(LOGNAME, "Starting\n");

    ESP_ERROR_CHECK(i2cdev_init());
    ESP_LOGI(LOGNAME, "i2c initialization complete.\n");

    hk_setup();
    xTaskCreatePinnedToCore(bmp280_loop, "bmp280_loop", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}
