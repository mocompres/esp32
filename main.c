/* i2c - Example
   Tis is  a test of the SSD1306 OLED display driver running in a task 
   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.
updated by Ole Schultz 18 okt. 2020
*/
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <string.h>
#include <driver/uart.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include "ssd1306.h"
#include "esp_heap_caps.h"
#include "bmp280.h" // sensor lib
#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define SCL_GPIO 5 //17                      //2                /*!< gpio number for I2C master clock */
#define SDA_GPIO 4 //16                      //14               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */

#define DEFAULT_FONT FONT_FACE_BITOCRbbA_6X11 //FONT_FACE_BITOCRA_4X7//FONT_FACE_BITOCRA_7X13//FONT_FACE_BITOCRA_4X7

#define LOAD_ICON_X 54
#define LOAD_ICON_Y 42
#define LOAD_ICON_SIZE 20

#define CIRCLE_COUNT_ICON_X 100
#define CIRCLE_COUNT_ICON_Y 52

#define SECOND (1000 / portTICK_PERIOD_MS)

// bmp280 stuff
#define SDA_GPIO_B 4
#define SCL_GPIO_B 5


SemaphoreHandle_t print_mux = NULL;
char tasklistbuf[20] = {0};

// Global variables for sensor 
float pressure, temperature, humidity;

char data[100];

// Display
static void ssd1306_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    unsigned int count = 0;
    char arr[20];
    //vTaskDelay(1000 / portTICK_PERIOD_MS);
    //InitializeDisplay();
    reset_display();
    while (1)
    {
        setXY(0, 0);
        sendStr("Sensor Data: "); 
        sprintf(arr, "Temperature : %.2f", temperature);
        sendStrXY(arr, 6, 0);
        sprintf(arr, "Pressure : %.2f", pressure);
        sendStrXY(arr, 4, 0);
        sprintf(arr, "Humidity : %.2f", humidity);
        sendStrXY(arr, 3, 0);
        
        
        sendCharXY('a', 2, 4);

        printf("%s: Started user interface task\n", __FUNCTION__);
        //vTaskDelay(SECOND);
        vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_RATE_MS));
    }
    vTaskDelete(NULL);
}

static void sensor_task(void *pvParameters) {

    
    printf("Sensort start\n");
   bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    
    printf("Sensor Error check\n");

    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, SDA_GPIO_B, SCL_GPIO_B));
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));


    printf("Sensor Error check done\n");

    bool bme280p = dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

    

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

void app_main(void)
{
    
    
    print_mux = xSemaphoreCreateMutex();
    i2c_example_master_init();
    ESP_ERROR_CHECK(i2cdev_init());

    printf("done\n");
    //InitializeDisplay();

    printf("done init display\n");

 //   xTaskCreate(ssd1306_task, " ssd1306_task", 2048, (void *)0, 2, NULL);
  
  // bmp280 task
  printf("Staring sensort\n");
  
  sensor_task((void*)0);

 // xTaskCreate(sensor_task, " sensor_task", 2048, (void *)0, 1, NULL);
}
