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
// keep the device in normal mode, to allow for consistent adjustment
// against PCB self-heating
const BMP280_Mode BMP280_UPDATE_MODE = BMP280_MODE_FORCED;
// moderate standby time in normal mode, ignored in forced mode
const BMP280_StandbyTime BMP280_STANDBY_TIME = BMP280_STANDBY_500;
 // amount we have to subtract to compensate for enclosure and PCB self-heating
const HkDecimal BMP280_TEMP_OFFSET = -3.7; // degrees C
const BMP280_Oversampling BMP280_PRESSURE_OVERSAMPLE = BMP280_SKIPPED;
const BMP280_Oversampling BMP280_HUMIDITY_OVERSAMPLE = BMP280_ULTRA_LOW_POWER;
const BMP280_Oversampling BMP280_TEMPERATURE_OVERSAMPLE = BMP280_ULTRA_LOW_POWER;
const BMP280_Filter BMP280_FILTER = BMP280_FILTER_OFF;
const uint16_t NOTIFY_EVERY = 60; // loops, 1/min

volatile HkDecimal pressure = 0;
volatile HkDecimal temperature = 0;
volatile HkDecimal humidity = 0;

void* chr_temperature_ptr = NULL;
void* chr_humidity_ptr = NULL;

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

esp_err_t safe_notify(void* chr)
{
    esp_err_t result = hk_notify(chr);
    if(result != ESP_OK)
    {
        ESP_LOGE(LOGNAME, "Failed to notify with error: %s\n", esp_err_to_name(result));
    }
    return result;
}

void bmp280_loop(void *pvParameters)
{
    uint16_t loopCount = 0;
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    params.mode = BMP280_UPDATE_MODE;
    params.oversampling_humidity = BMP280_HUMIDITY_OVERSAMPLE;
    params.oversampling_temperature = BMP280_TEMPERATURE_OVERSAMPLE;
    params.oversampling_pressure = BMP280_PRESSURE_OVERSAMPLE;
    params.filter = BMP280_FILTER;
    params.standby = BMP280_STANDBY_TIME;

    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_1, 1, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));

    bool bme280p = dev.id == BME280_CHIP_ID;
    ESP_LOGI(LOGNAME, "Found %s\n", bme280p ? "BME280" : "BMP280");

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(250));
        float fTemp, fPres, fHum;
        bool measuring = true;

        if(BMP280_UPDATE_MODE == BMP280_MODE_FORCED)
        {
            if( bmp280_force_measurement(&dev) != ESP_OK )
            {
                ESP_LOGW(LOGNAME, "Force measurement failed; waiting 250ms then retrying...");
                continue;
            }
            while(measuring) 
            {
                if( bmp280_is_measuring(&dev, &measuring) != ESP_OK )
                {
                    ESP_LOGW(LOGNAME, "Cannot resolve if measuring! Waiting...");
                }
                else if (measuring)
                {
                    ESP_LOGD(LOGNAME, "Measuring; waiting...");
                }
                else
                {
                    ESP_LOGD(LOGNAME, "Measuring complete!");
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        if (bmp280_read_float(&dev, &fTemp, &fPres, &fHum) != ESP_OK)
        {
            ESP_LOGW(LOGNAME, "Temperature/pressure reading failed; waiting 250ms then retrying...");
            continue;
        }
        loopCount += 1;
        pressure = (HkDecimal)fPres;
        temperature = (HkDecimal)(fTemp + BMP280_TEMP_OFFSET);
        humidity = (HkDecimal)fHum;
        ESP_LOGI(LOGNAME, "T:%.2fC H:%.2f", temperature, humidity);
        if(loopCount == NOTIFY_EVERY)
        {
            if(chr_humidity_ptr != NULL)  safe_notify(chr_humidity_ptr);
            if(chr_temperature_ptr != NULL) safe_notify(chr_temperature_ptr);
            loopCount = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void hk_setup(void)
{
    ESP_ERROR_CHECK(hk_setup_start());
    ESP_ERROR_CHECK(hk_setup_add_accessory("Enviroment Sensor", "Conrad's Closet", "ESP32 D1 Mini", "0000001", "1", on_identify));
    ESP_ERROR_CHECK(hk_setup_add_srv(HK_SRV_TEMPERATURE_SENSOR, true, false));
    ESP_ERROR_CHECK(hk_setup_add_chr(HK_CHR_CURRENT_TEMPERATURE, on_read_temp, NULL, true, &chr_temperature_ptr)); 
    
    ESP_ERROR_CHECK(hk_setup_add_srv(HK_SRV_HUMIDITY_SENSOR, true, false));
    ESP_ERROR_CHECK(hk_setup_add_chr(HK_CHR_CURRENT_RELATIVE_HUMIDITY, on_read_humid, NULL, true, &chr_humidity_ptr)); 
    ESP_ERROR_CHECK(hk_setup_finish());

    ESP_ERROR_CHECK(hk_init("ConSense", HK_CAT_SENSOR, "026-62-932"));
    ESP_LOGI(LOGNAME, "HK initialization complete.\n");
}

esp_err_t pm_init(void)
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
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(LOGNAME, "SDK version:%s\n", esp_get_idf_version());
    ESP_LOGI(LOGNAME, "Starting\n");
    ESP_ERROR_CHECK(pm_init());
    ESP_LOGI(LOGNAME, "Power management initialized.\n");

    ESP_ERROR_CHECK(i2cdev_init());
    ESP_LOGI(LOGNAME, "i2c initialization complete.\n");

    hk_setup();
    xTaskCreatePinnedToCore(bmp280_loop, "bmp280_loop", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}
