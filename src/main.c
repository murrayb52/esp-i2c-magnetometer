#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"

// ---------------------------------------
// Defines
// ---------------------------------------
#define LOOP_WAIT_TIME_MS       1000
#define I2C_CLOCK_FREQ_HZ       400000
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_SCL_IO       22
#define MAGN_I2C_PORT           0
#define MAGN_TIMEOUT_MS         500

#define GY271_ADDR              0x2C
#define REG_X_DATA_LSB          0x00
#define REG_Y_DATA_LSB          0x02
#define REG_Z_DATA_LSB          0x04
#define REG_BYTES_PER_AXIS      0x02
#define REG_CONTROL_REG_1       0x09

// ---------------------------------------
// Type defines
// ---------------------------------------
typedef enum {
    TASK_ID_MAGNETOMETER = 0,
} taskIdEnum;

typedef enum {
    PRIORITY_LOW = 0,
    PRIORITY_MED,
    PRIORITY_HI,
} priorityEnum;

typedef struct {
    uint16_t xReading;
    uint16_t yReading;
    uint16_t zReading;
} magnRawData_t;

// ---------------------------------------
// Variables
// ---------------------------------------
static const char *TAG = "MAGN";
TaskHandle_t magSvcTask = 0;

i2c_master_bus_config_t i2cMasterConfig = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = MAGN_I2C_PORT,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

i2c_device_config_t magnDevConfig = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = GY271_ADDR,
    .scl_speed_hz = 100000,
};

i2c_master_bus_handle_t i2cBusHandle = NULL;
i2c_master_dev_handle_t magnDeviceHandle = NULL;

// ---------------------------------------
// Functions declarations
// ---------------------------------------
void scanBus(void);
void initMagn(void);
void magnSvc(void *pvParameters);
void readXYZData(magnRawData_t* magnData);
void printXYZData(magnRawData_t* magnData);
void printAllData(uint8_t* allData, uint8_t printBytes);
void readAllMagnData(uint8_t* allData, uint8_t readBytes);
void configureMagnDevice(void);
void forceReadAxisAddresses(void);

// ---------------------------------------
// Global Functions
// ---------------------------------------
void app_main(void) {

    initMagn();
    xTaskCreatePinnedToCore(magnSvc, "magnetometerSvc", 4096, (void*)TASK_ID_MAGNETOMETER, PRIORITY_LOW, NULL, tskNO_AFFINITY);
}


// ---------------------------------------
// Local Functions
// ---------------------------------------
void scanBus(void) {
    ESP_LOGI(TAG, "Starting I2C bus scan...");
    ESP_LOGI(TAG, "Scanning addresses 0x08 to 0x77...");
    
    uint8_t devicesFound = 0;
    
    for (uint16_t addr = 0x08; addr <= 0x77; addr++) {
        esp_err_t ret = i2c_master_probe(i2cBusHandle, addr, MAGN_TIMEOUT_MS);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  >> Device found at address: 0x%02X", addr);
            devicesFound++;
        }
    }
    
    if (devicesFound == 0) {
        ESP_LOGW(TAG, "No I2C devices found on the bus!");
    } else {
        ESP_LOGI(TAG, "I2C scan complete. Found %d device(s).", devicesFound);
    }
    ESP_LOGI(TAG, "");
}

void initMagn(void) {
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    // I2C Bus Initialisation
    i2c_new_master_bus(&i2cMasterConfig, &i2cBusHandle);
    
    // Scan the I2C bus to find all responding devices
    scanBus();
    
    // Probe target device and ensure it is ACKd
    ESP_ERROR_CHECK(i2c_master_probe(i2cBusHandle, magnDevConfig.device_address, MAGN_TIMEOUT_MS));
    ESP_LOGI(TAG,"GY-271 Found.");
    
    // Initialise GY-271
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cBusHandle, &magnDevConfig, &magnDeviceHandle));
    ESP_LOGI(TAG, "Initialised GY-271 Magnetometer.");

    // Set continuous mode
    configureMagnDevice();

    forceReadAxisAddresses();
}

void magnSvc(void *pvParameters) {
    (void)pvParameters;
    
    while(1) {
        // Check if data ready
        uint8_t statusAddr = 0x06;
        uint8_t statusValue = 0;
        i2c_master_transmit_receive(magnDeviceHandle, &statusAddr, 1, &statusValue, 1, MAGN_TIMEOUT_MS);
        ESP_LOGI(TAG, "Status: 0x%02X (Bit 0 = data ready)", statusValue);

        uint8_t readBytes = 0x0C;
        uint8_t readAllData[readBytes];
        readAllMagnData(readAllData, readBytes);
        printAllData(readAllData, readBytes);
        
        //magnRawData_t magnData = {0};
        //readXYZData(&magnData);
        //printXYZData(&magnData);
        vTaskDelay(LOOP_WAIT_TIME_MS / portTICK_PERIOD_MS);
    }
}

void configureMagnDevice(void) {
    esp_err_t ret;
    uint8_t regValue = 0;
    uint8_t regAddr;
    
    // Official datasheet initialization sequence (Section 7.1 Application Examples)
    // NO soft reset - write SET/RESET period first, then control register
    
    // Step 1: Write SET/RESET Period register (0x0B = 0x01)
    uint8_t setPeriod[2] = {0x0B, 0x01};
    ret = i2c_master_transmit(magnDeviceHandle, setPeriod, 2, MAGN_TIMEOUT_MS);
    ESP_LOGI(TAG, "Write 0x0B=0x01 (SET/RESET Period): %s", esp_err_to_name(ret));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Step 2: Write Control Register (0x09)
    // Value: 0x1D = 0b00011101 = MODE=01 (Continuous), ODR=11 (200Hz), RNG=10 (8G), OSR=01 (512)
    uint8_t controlReg[2] = {0x09, 0x1D};
    ret = i2c_master_transmit(magnDeviceHandle, controlReg, 2, MAGN_TIMEOUT_MS);
    ESP_LOGI(TAG, "Write 0x09=0x1D (Control - Continuous Mode): %s", esp_err_to_name(ret));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Verify configuration
    regAddr = 0x09;
    i2c_master_transmit_receive(magnDeviceHandle, &regAddr, 1, &regValue, 1, MAGN_TIMEOUT_MS);
    ESP_LOGI(TAG, "Reg 0x09 read back: 0x%02X (should be 0x1D)", regValue);
    
    regAddr = 0x0B;
    i2c_master_transmit_receive(magnDeviceHandle, &regAddr, 1, &regValue, 1, MAGN_TIMEOUT_MS);
    ESP_LOGI(TAG, "Reg 0x0B read back: 0x%02X (should be 0x01)", regValue);
}

void readAllMagnData(uint8_t* allData, uint8_t readBytes) {
    uint8_t startAddress = REG_X_DATA_LSB;
    uint8_t readBuffer[readBytes];
    
    // Bulk read: Set register pointer to 0x00, then read multiple bytes (auto-increment)
    esp_err_t result = i2c_master_transmit_receive(
        magnDeviceHandle,
        &startAddress,
        sizeof(startAddress),
        readBuffer,
        readBytes,
        MAGN_TIMEOUT_MS * 10
    );

    if (result == ESP_OK) {
        // Copy data from readBuffer to allData output
        for (uint8_t i = 0; i < readBytes; i++) {
            allData[i] = readBuffer[i];
        }
    }
    else {
        ESP_LOGW(TAG, "Failed bulk read from 0x%02X: %s", startAddress, esp_err_to_name(result));
    }
}

void forceReadAxisAddresses(void) {
    esp_err_t ret;
    uint8_t regAddr;
    uint8_t regValue;
    
    ESP_LOGI(TAG, "Reading each axis register individually:");
    
    // Read X-axis LSB (0x00)
    regAddr = 0x00;
    ret = i2c_master_transmit_receive(magnDeviceHandle, &regAddr, 1, &regValue, 1, MAGN_TIMEOUT_MS);
    ESP_LOGI(TAG, "  X LSB (0x00): 0x%02X - %s", regValue, esp_err_to_name(ret));
    
    // Read X-axis MSB (0x01)
    regAddr = 0x01;
    ret = i2c_master_transmit_receive(magnDeviceHandle, &regAddr, 1, &regValue, 1, MAGN_TIMEOUT_MS);
    ESP_LOGI(TAG, "  X MSB (0x01): 0x%02X - %s", regValue, esp_err_to_name(ret));
    
    // Read Y-axis LSB (0x02)
    regAddr = 0x02;
    ret = i2c_master_transmit_receive(magnDeviceHandle, &regAddr, 1, &regValue, 1, MAGN_TIMEOUT_MS);
    ESP_LOGI(TAG, "  Y LSB (0x02): 0x%02X - %s", regValue, esp_err_to_name(ret));
    
    // Read Y-axis MSB (0x03)
    regAddr = 0x03;
    ret = i2c_master_transmit_receive(magnDeviceHandle, &regAddr, 1, &regValue, 1, MAGN_TIMEOUT_MS);
    ESP_LOGI(TAG, "  Y MSB (0x03): 0x%02X - %s", regValue, esp_err_to_name(ret));
    
    // Read Z-axis LSB (0x04)
    regAddr = 0x04;
    ret = i2c_master_transmit_receive(magnDeviceHandle, &regAddr, 1, &regValue, 1, MAGN_TIMEOUT_MS);
    ESP_LOGI(TAG, "  Z LSB (0x04): 0x%02X - %s", regValue, esp_err_to_name(ret));
    
    // Read Z-axis MSB (0x05)
    regAddr = 0x05;
    ret = i2c_master_transmit_receive(magnDeviceHandle, &regAddr, 1, &regValue, 1, MAGN_TIMEOUT_MS);
    ESP_LOGI(TAG, "  Z MSB (0x05): 0x%02X - %s", regValue, esp_err_to_name(ret));
    
    ESP_LOGI(TAG, "Individual axis reads complete.\n");
}

void readXYZData(magnRawData_t* magnData) {
    uint8_t xAxisLsb = REG_X_DATA_LSB;
    
    const uint8_t readBytes = 0x06;
    uint8_t readBuffer[readBytes];
    
    // write to Magn to retrieve XYZ data
    esp_err_t result = i2c_master_transmit_receive(
        magnDeviceHandle,
        &xAxisLsb,
        sizeof(xAxisLsb),
        readBuffer,
        readBytes,
        MAGN_TIMEOUT_MS
    );

    if (result == ESP_OK) {
        magnData->xReading = ((uint16_t)(readBuffer[0] << 0x01)) | ((uint16_t)(readBuffer[1]));
        magnData->yReading = ((uint16_t)(readBuffer[2] << 0x01)) | ((uint16_t)(readBuffer[3]));
        magnData->zReading = ((uint16_t)(readBuffer[4] << 0x01)) | ((uint16_t)(readBuffer[5]));
    }
    else {
        ESP_LOGW(TAG, "Failed to read XYZ data.");
    }
}

void printXYZData(magnRawData_t* magnData) {
    ESP_LOGI(TAG, "MAGN [ X: %d, Y: %d, Z: %d ]", magnData->xReading, magnData->yReading, magnData->zReading);
}

void printAllData(uint8_t* allData, uint8_t printBytes) {
    ESP_LOGI(TAG, "Bulk read results (assuming auto-increment from 0x00):");
    for (uint8_t byte = 0; byte < printBytes; byte++) {
        uint8_t byteValue = allData[byte];
        ESP_LOGI(TAG, "  [Index %d -> Reg 0x%02X]: 0x%02X (%d)", byte, byte, byteValue, byteValue);
    }
    ESP_LOGI(TAG,"");
}