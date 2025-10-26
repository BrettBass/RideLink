#include "Compass.hpp"
#include "esp_log.h"
#include "soc/gpio_num.h"
#include <cmath>

static const char *TAG = "COMPASS";

// I2C Configuration
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   GPIO_NUM_21
#define I2C_MASTER_SCL_IO   GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ  100000
#define I2C_MASTER_TIMEOUT_MS 1000

Compass::Compass()
    : i2c_port(I2C_MASTER_NUM)
    , declination(0.0f)
    , device_address(QMC5883L_ADDR)  // Default to QMC
    , is_qmc(true)
    , x_offset(0)
    , y_offset(0)
    , z_offset(0)
    , x_scale(1.0f)
    , y_scale(1.0f)
    , z_scale(1.0f)
    , x_min(32767), x_max(-32768)
    , y_min(32767), y_max(-32768)
    , z_min(32767), z_max(-32768)
    , calibrating(false)
{
}

Compass::~Compass()
{
    i2c_driver_delete(i2c_port);
}

bool Compass::init()
{
    ESP_LOGI(TAG, "Initializing compass (QMC5883L/HMC5883L)");

    // Configure I2C
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;

    esp_err_t ret = i2c_param_config(i2c_port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(ret));
        return false;
    }

    ret = i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return false;
    }

    // Try standard QMC5883L address first (0x0D), then fallback to 0x2C and HMC (0x1E)
    uint8_t addresses[] = {0x0D, 0x2C, 0x1E};
    bool found = false;

    for (int i = 0; i < 3; i++) {
        device_address = addresses[i];
        uint8_t dummy;
        if (readRegister(0x00, dummy)) {
            ESP_LOGI(TAG, "Found compass at address 0x%02X", device_address);
            found = true;
            break;
        }
    }

    if (!found) {
        ESP_LOGE(TAG, "Cannot communicate with compass at any address");
        return false;
    }

    // Initialize QMC5883L
    ESP_LOGI(TAG, "Initializing QMC5883L at 0x%02X", device_address);

    bool initialized = false;

    // AGGRESSIVE RESET SEQUENCE
    ESP_LOGI(TAG, "Performing aggressive reset sequence...");

    // Multiple reset attempts with different timings
    for (int reset_attempt = 0; reset_attempt < 3; reset_attempt++) {
        // Soft reset
        writeRegister(0x0A, 0x80);
        vTaskDelay(pdMS_TO_TICKS(200));

        // Write standby mode
        writeRegister(0x09, 0x00);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Clear SET/RESET period
        writeRegister(0x0B, 0x01);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Configuration attempts with different register values
    struct {
        uint8_t ctrl1;
        uint8_t ctrl2;
        uint8_t period;
        const char* desc;
    } configs[] = {
        // Try simpler configs first
        {0x01, 0x00, 0x01, "10Hz, 2G, 512 OSR, Continuous"},
        {0x05, 0x00, 0x01, "50Hz, 2G, 512 OSR, Continuous"},
        {0x09, 0x00, 0x01, "100Hz, 2G, 512 OSR, Continuous"},
        {0x0D, 0x00, 0x01, "200Hz, 2G, 512 OSR, Continuous"},
        // Try with different OSR
        {0x19, 0x00, 0x01, "100Hz, 2G, 256 OSR, Continuous"},
        {0x39, 0x00, 0x01, "100Hz, 2G, 128 OSR, Continuous"},
        {0x59, 0x00, 0x01, "100Hz, 2G, 64 OSR, Continuous"},
        // Try 8G range
        {0x11, 0x00, 0x01, "10Hz, 8G, 512 OSR, Continuous"},
        {0x1D, 0x00, 0x01, "200Hz, 8G, 512 OSR, Continuous"},
        // Try with SET/RESET enabled
        {0x01, 0x01, 0x01, "10Hz, 2G, 512 OSR, Continuous + SET/RESET"},
        {0x09, 0x01, 0x01, "100Hz, 2G, 512 OSR, Continuous + SET/RESET"},
    };

    int i = 9;
        ESP_LOGI(TAG, "Trying config %d: %s", i+1, configs[i].desc);

        // Multiple write attempts for each config
        for (int write_attempt = 0; write_attempt < 2; write_attempt++) {
            // Write SET/RESET period first
            writeRegister(0x0B, configs[i].period);
            vTaskDelay(pdMS_TO_TICKS(10));

            // Write control register 2
            writeRegister(0x0A, configs[i].ctrl2);
            vTaskDelay(pdMS_TO_TICKS(10));

            // Write control register 1 (starts measurement)
            writeRegister(0x09, configs[i].ctrl1);
            vTaskDelay(pdMS_TO_TICKS(200));  // Longer delay after starting measurement

            // Try to read data multiple times with varying delays
            for (int attempt = 0; attempt < 10; attempt++) {
                // Check status register first
                uint8_t status;
                if (readRegister(0x06, status)) {
                    ESP_LOGI(TAG, "  Status reg: 0x%02X (DRDY=%d, OVL=%d, DOR=%d)",
                            status, (status & 0x01), (status & 0x02) >> 1, (status & 0x04) >> 2);
                }

                uint8_t data[6];
                if (readRegisters(0x00, data, 6)) {
                    int16_t x = (int16_t)((data[1] << 8) | data[0]);
                    int16_t y = (int16_t)((data[3] << 8) | data[2]);
                    int16_t z = (int16_t)((data[5] << 8) | data[4]);

                    ESP_LOGI(TAG, "  Attempt %d: X=%d, Y=%d, Z=%d", attempt+1, x, y, z);

                    // Check if we're getting real data (not stuck values)
                    if ((x != 128 || y != 0 || z != 0) &&
                        (x != 0 || y != 0 || z != 0) &&
                        (x != -1 || y != -1 || z != -1) &&
                        (abs(x) > 10 || abs(y) > 10 || abs(z) > 10)) {  // Must have some magnitude
                        ESP_LOGI(TAG, "*** SUCCESS! Config %d works! ***", i+1);
                        initialized = true;
                        break;
                    }
                }

                // Vary delay times
                if (attempt < 3) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                } else if (attempt < 6) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                } else {
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
            }
        }


    if (!initialized) {
        ESP_LOGW(TAG, "================================================");
        ESP_LOGW(TAG, "Standard configs didn't work!");
        ESP_LOGW(TAG, "This could mean:");
        ESP_LOGW(TAG, "1. Module is defective");
        ESP_LOGW(TAG, "2. Module needs 5V power (not 3.3V)");
        ESP_LOGW(TAG, "3. Module is a different chip variant");
        ESP_LOGW(TAG, "4. Wiring issue (check SDA/SCL)");
        ESP_LOGW(TAG, "================================================");
    }

    // Dump registers for debugging
    ESP_LOGI(TAG, "Register dump:");
    for (uint8_t reg = 0x00; reg <= 0x0D; reg++) {
        uint8_t val;
        if (readRegister(reg, val)) {
            ESP_LOGI(TAG, "  Reg[0x%02X] = 0x%02X (%d)", reg, val, val);
        }
    }

    // Final test
    int16_t x, y, z;
    if (readRaw(x, y, z)) {
        ESP_LOGI(TAG, "Final test read: X=%d, Y=%d, Z=%d", x, y, z);
    }

    return true;  // Return true anyway so we can see debug output
}

bool Compass::isConnected()
{
    uint8_t dummy;
    return readRegister(0x00, dummy);
}

bool Compass::readRaw(int16_t &x, int16_t &y, int16_t &z)
{
    uint8_t data[6];

    // Read data registers
    if (!readRegisters(0x00, data, 6)) {
        // Try reading one by one if bulk read fails
        for (int i = 0; i < 6; i++) {
            if (!readRegister(i, data[i])) {
                return false;
            }
        }
    }

    // Parse data - LSB first for QMC5883L
    x = (int16_t)((data[1] << 8) | data[0]);
    y = (int16_t)((data[3] << 8) | data[2]);
    z = (int16_t)((data[5] << 8) | data[4]);

    // Debug: Log raw bytes occasionally
    static int debug_counter = 0;
    if (debug_counter++ % 100 == 0) {
        ESP_LOGI(TAG, "Raw bytes: %02X %02X %02X %02X %02X %02X -> X=%d Y=%d Z=%d",
                data[0], data[1], data[2], data[3], data[4], data[5],
                x, y, z);
    }

    // Don't apply calibration if we're getting stuck values
    if (x == 128 && y == 0 && z == 0) {
        return false;
    }

    // During calibration, update min/max values with raw data BEFORE applying calibration
    if (calibrating) {
        updateCalibration(x, y, z);
        // Return raw values during calibration
        return true;
    }

    // Apply calibration only after calibration is complete
    x = (int16_t)((x - x_offset) * x_scale);
    y = (int16_t)((y - y_offset) * y_scale);
    z = (int16_t)((z - z_offset) * z_scale);

    return true;
}

float Compass::getHeading()
{
    int16_t x, y, z;

    if (!readRaw(x, y, z)) {
        return 0.0f;
    }

    // Calculate heading using calibrated values
    // NOTE: Using atan2(x, y) to match working Arduino implementation
    // This gives heading where device's Y-axis points North when heading = 0
    float heading = atan2f((float)x, (float)y);

    // Convert to degrees
    heading = heading * 180.0f / M_PI;

    // Apply declination (already in degrees)
    heading += declination;

    // Normalize to 0-359 degrees
    while (heading < 0) {
        heading += 360.0f;
    }
    while (heading >= 360.0f) {
        heading -= 360.0f;
    }

    return heading;
}

void Compass::setDeclination(float decl)
{
    declination = decl;
    ESP_LOGI(TAG, "Magnetic declination set to %.2f degrees", declination);
}

void Compass::startCalibration()
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "COMPASS CALIBRATION STARTING");
    ESP_LOGI(TAG, "Rotate the device in ALL directions:");
    ESP_LOGI(TAG, "  - Rotate around all 3 axes");
    ESP_LOGI(TAG, "  - Make figure-8 patterns");
    ESP_LOGI(TAG, "  - Tilt in all directions");
    ESP_LOGI(TAG, "==============================================");

    calibrating = true;

    // Reset min/max values
    x_min = y_min = z_min = 32767;
    x_max = y_max = z_max = -32768;

    // Reset offsets and scales during calibration
    x_offset = y_offset = z_offset = 0;
    x_scale = y_scale = z_scale = 1.0f;
}

void Compass::updateCalibration(int16_t x, int16_t y, int16_t z)
{
    if (!calibrating) return;

    // Track min/max for each axis
    if (x < x_min) x_min = x;
    if (x > x_max) x_max = x;
    if (y < y_min) y_min = y;
    if (y > y_max) y_max = y;
    if (z < z_min) z_min = z;
    if (z > z_max) z_max = z;

    // Show progress every so often
    static int update_counter = 0;
    if (++update_counter % 10 == 0) {
        ESP_LOGI(TAG, "Calibrating... X[%d,%d] Y[%d,%d] Z[%d,%d]",
                x_min, x_max, y_min, y_max, z_min, z_max);
    }
}

void Compass::finishCalibration()
{
    if (!calibrating) return;

    // Calculate offsets (center point of each axis range)
    x_offset = (x_min + x_max) / 2;
    y_offset = (y_min + y_max) / 2;
    z_offset = (z_min + z_max) / 2;

    // Calculate scales (to normalize each axis to same range)
    int16_t x_range = x_max - x_min;
    int16_t y_range = y_max - y_min;
    int16_t z_range = z_max - z_min;

    // Find the average range
    int16_t avg_range = (x_range + y_range + z_range) / 3;

    // Calculate scale factors to normalize to average range
    if (x_range > 0) x_scale = (float)avg_range / x_range;
    if (y_range > 0) y_scale = (float)avg_range / y_range;
    if (z_range > 0) z_scale = (float)avg_range / z_range;

    calibrating = false;

    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "CALIBRATION COMPLETE!");
    ESP_LOGI(TAG, "Offsets: X=%d, Y=%d, Z=%d", x_offset, y_offset, z_offset);
    ESP_LOGI(TAG, "Scales: X=%.3f, Y=%.3f, Z=%.3f", x_scale, y_scale, z_scale);
    ESP_LOGI(TAG, "Ranges: X=%d, Y=%d, Z=%d", x_range, y_range, z_range);
    ESP_LOGI(TAG, "==============================================");

    // Save these values - you could store them in NVS flash
    ESP_LOGI(TAG, "Add these to your code for permanent calibration:");
    ESP_LOGI(TAG, "compass.setCalibrationOffsets(%d, %d, %d);", x_offset, y_offset, z_offset);
    ESP_LOGI(TAG, "compass.setCalibrationScales(%.3f, %.3f, %.3f);", x_scale, y_scale, z_scale);
}

void Compass::setCalibrationOffsets(int16_t x, int16_t y, int16_t z)
{
    x_offset = x;
    y_offset = y;
    z_offset = z;
    ESP_LOGI(TAG, "Calibration offsets set: X=%d, Y=%d, Z=%d", x_offset, y_offset, z_offset);
}

void Compass::setCalibrationScales(float x, float y, float z)
{
    x_scale = x;
    y_scale = y;
    z_scale = z;
    ESP_LOGI(TAG, "Calibration scales set: X=%.3f, Y=%.3f, Z=%.3f", x_scale, y_scale, z_scale);
}


bool Compass::writeRegister(uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg, esp_err_to_name(ret));
    }

    return (ret == ESP_OK);
}

bool Compass::readRegister(uint8_t reg, uint8_t &value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);  // Repeated start
    i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK);
}

bool Compass::readRegisters(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);  // Repeated start
    i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_READ, true);

    for (size_t i = 0; i < len - 1; i++) {
        i2c_master_read_byte(cmd, &data[i], I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK);
}
