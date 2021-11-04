#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <bmp280.h>
#include <string.h>
#include "ssd1306.h"

//#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 21
#define SCL_GPIO 22

#define SCL_GPIO_S 22                      //2                /*!< gpio number for I2C master clock */
#define SDA_GPIO_S 21                      //14               /*!< gpio number for I2C master data  */

//#else
//#define SDA_GPIO 16
//#define SCL_GPIO 17
//#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

/* esp_err_t i2c_example_master_init()
{
   
i2c_config_t conf;

conf.mode = I2C_MODE_MASTER;

conf.sda_io_num = SDA_GPIO_S;  //14

conf.scl_io_num =SCL_GPIO_S;   //2

conf.sda_pullup_en = GPIO_PULLUP_ENABLE;

conf.scl_pullup_en = GPIO_PULLUP_ENABLE;

conf.master.clk_speed = 1000000;
//conf.clk_flags=0;



// install the driver

ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode,0,0,0));

printf("- i2c driver installed\r\n\r\n");

ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));

printf("- i2c controller configured\r\n");

printf("scanning the bus...\r\n\r\n");

int devices_found = 0;



for(int address = 1; address < 127; address++) {



	// create and execute the command link

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);

	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);

	i2c_master_stop(cmd);

	if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS) == ESP_OK) {
        // dev.i2c_addr=address;
		printf("-> found device with address 0x%02x\r\n", address);

		devices_found++;

	}

	i2c_cmd_link_delete(cmd);

}

if(devices_found == 0) printf("\r\n-> no devices found\r\n");

printf("\r\n...scan completed!\r\n");

   return ESP_OK;
} */

float pressure = 0.0f, temperature = 0.0f, humidity = 0.0f;

void bmp280_test(void *pvParameters)
{
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, SDA_GPIO, SCL_GPIO_S));
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));

    bool bme280p = dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

   // float pressure, temperature, humidity;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }

        /* float is used in printf(). you need non-default configuration in
         * sdkconfig for ESP8266, which is enabled by default for this
         * example. see sdkconfig.defaults.esp8266
         */
        printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
        if (bme280p)
            printf(", Humidity: %.2f\n", humidity);
        else
            printf("\n");
    }
}


// Display
static void ssd1306_task(void *pvParameters)
{

 //   float pressure = 0.0f, temperature = 0.0f, humidity = 0.0f; // placeholder before global 
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    unsigned int count = 0;
    char arr[20];
    //vTaskDelay(1000 / portTICK_PERIOD_MS);
    InitializeDisplay();
    reset_display();
    while (1)
    {
        setXY(0, 0);
        sendStr("Sensor Data: "); 
        sprintf(arr, "Temperature : %.2f", temperature);
        sendStrXY(arr, 6, 0);
        sprintf(arr, "Pressure : %.2f", pressure);
        sendStrXY(arr, 4, 0);
     //   sprintf(arr, "Humidity : %.2f", humidity);
     //   sendStrXY(arr, 3, 0);
        
        
     //   sendCharXY('a', 2, 4);

        printf("%s: Started user interface task\n", __FUNCTION__);
        //vTaskDelay(SECOND);
        vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_RATE_MS));
    }
    vTaskDelete(NULL);
}


void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());
   //xTaskCreatePinnedToCore(bmp280_test, "bmp280_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

    //bmp280_test((void *) 0);
    xTaskCreate(bmp280_test, " bmp280_test", 2048, (void *)0, 2, NULL);
    xTaskCreate(ssd1306_task, " ssd1306_task", 2048, (void *)0, 1, NULL);
}
