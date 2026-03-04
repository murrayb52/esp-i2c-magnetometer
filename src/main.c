#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
// ---------------------------------------
// Typedef
// ---------------------------------------
typedef enum {
    TASK_ID_MAGNETOMETER = 0,
} taskIdEnum;

typedef enum {
    PRIORITY_LOW = 0,
    PRIORITY_MED,
    PRIORITY_HI,
} priorityEnum;

// ---------------------------------------
// Variables and defines
// ---------------------------------------
TaskHandle_t magSvcTask = 0;


// ---------------------------------------
// Functions declarations
// ---------------------------------------
void getMagValues(void *pvParameters);


// ---------------------------------------
// Global Functions
// ---------------------------------------
void app_main(void) {

    xTaskCreatePinnedToCore(getMagValues, "magnetometerSvc", 4096, (void*)TASK_ID_MAGNETOMETER, PRIORITY_LOW, NULL, tskNO_AFFINITY);
}


// ---------------------------------------
// Local Functions
// ---------------------------------------
void getMagValues(void *pvParameters) {
    (void)pvParameters;

    while(1) {

        ESP_LOGI("MAG", "Hello World!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}