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
    ESP_LOGI(TAG, "Initializing compass (QMC5883L)");

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

    // Try to find compass at common addresses
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
        ESP_LOGE(TAG, "Cannot find compass! Check wiring: SDA=GPIO21, SCL=GPIO22");
        return false;
    }

    // Reset sequence
    ESP_LOGI(TAG, "Resetting compass...");

    // Soft reset
    writeRegister(0x0A, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Standby mode
    writeRegister(0x09, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Clear SET/RESET period
    writeRegister(0x0B, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Use the working configuration (config index 9 from your test)
    // This is: "10Hz, 2G, 512 OSR, Continuous + SET/RESET"
    ESP_LOGI(TAG, "Applying working configuration: 10Hz, 2G, 512 OSR, Continuous + SET/RESET");

    // Write SET/RESET period register
    writeRegister(0x0B, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Write control register 2 (enable SET/RESET)
    writeRegister(0x0A, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Write control register 1 (10Hz, 2G, 512 OSR, Continuous mode)
    writeRegister(0x09, 0x01);
    vTaskDelay(pdMS_TO_TICKS(200));

    // Verify the compass is working
    ESP_LOGI(TAG, "Verifying compass operation...");

    int good_reads = 0;
    for (int i = 0; i < 5; i++) {
        int16_t x, y, z;
        uint8_t data[6];
        if (readRegisters(0x00, data, 6)) {
            x = (int16_t)((data[1] << 8) | data[0]);
            y = (int16_t)((data[3] << 8) | data[2]);
            z = (int16_t)((data[5] << 8) | data[4]);

            // Check for valid data
            if ((x != 0 || y != 0 || z != 0) &&
                (abs(x) > 5 || abs(y) > 5 || abs(z) > 5)) {
                ESP_LOGI(TAG, "Read %d OK: X=%d, Y=%d, Z=%d", i+1, x, y, z);
                good_reads++;
            } else {
                ESP_LOGW(TAG, "Read %d suspicious: X=%d, Y=%d, Z=%d", i+1, x, y, z);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (good_reads >= 3) {
        ESP_LOGI(TAG, "✓ Compass initialized successfully!");
    } else {
        ESP_LOGW(TAG, "⚠ Compass may not be working properly");
    }

    return true;
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
        return false;
    }

    // Parse data - LSB first for QMC5883L
    x = (int16_t)((data[1] << 8) | data[0]);
    y = (int16_t)((data[3] << 8) | data[2]);
    z = (int16_t)((data[5] << 8) | data[4]);

    // Check for invalid/stuck values
    if ((x == 0 && y == 0 && z == 0) ||
        (x == -1 && y == -1 && z == -1) ||
        (abs(x) < 5 && abs(y) < 5 && abs(z) < 5)) {
        // Data looks suspicious, try reading again
        vTaskDelay(pdMS_TO_TICKS(10));

        if (!readRegisters(0x00, data, 6)) {
            return false;
        }

        x = (int16_t)((data[1] << 8) | data[0]);
        y = (int16_t)((data[3] << 8) | data[2]);
        z = (int16_t)((data[5] << 8) | data[4]);

        // Still bad? Give up
        if ((x == 0 && y == 0 && z == 0) ||
            (abs(x) < 5 && abs(y) < 5 && abs(z) < 5)) {
            return false;
        }
    }

    // During calibration, update min/max with raw values
    if (calibrating) {
        updateCalibration(x, y, z);
        return true;  // Return raw values during calibration
    }

    // Apply calibration offsets and scales
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

    // Calculate heading
    // Use atan2(y, x) for standard orientation where:
    // - X points North (0°)
    // - Y points East (90°)
    float heading = atan2f((float)y, (float)x);

    // Convert to degrees
    heading = heading * 180.0f / M_PI;

    // Apply magnetic declination
    heading += declination;

    // Normalize to 0-359 degrees
    if (heading < 0) {
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
    ESP_LOGI(TAG, "Rotate the device slowly in ALL directions:");
    ESP_LOGI(TAG, "  - Spin 360° horizontally (flat)");
    ESP_LOGI(TAG, "  - Tilt forward/backward");
    ESP_LOGI(TAG, "  - Roll left/right");
    ESP_LOGI(TAG, "  - Make slow figure-8 patterns");
    ESP_LOGI(TAG, "==============================================");

    calibrating = true;

    // Reset min/max values
    x_min = y_min = z_min = 32767;
    x_max = y_max = z_max = -32768;

    // Reset calibration values during calibration
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

    // Show progress
    static int update_counter = 0;
    if (++update_counter % 10 == 0) {
        ESP_LOGI(TAG, "Calibrating... X[%d to %d] Y[%d to %d] Z[%d to %d]",
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

    // Calculate ranges
    int16_t x_range = x_max - x_min;
    int16_t y_range = y_max - y_min;
    int16_t z_range = z_max - z_min;

    // Find the average range (for normalization)
    int16_t avg_range = (x_range + y_range + z_range) / 3;

    // Calculate scale factors to normalize each axis to the average range
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
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "SAVE THESE VALUES in your code:");
    ESP_LOGI(TAG, "compass.setCalibrationOffsets(%d, %d, %d);", x_offset, y_offset, z_offset);
    ESP_LOGI(TAG, "compass.setCalibrationScales(%.2f, %.2f, %.2f);", x_scale, y_scale, z_scale);
    ESP_LOGI(TAG, "");
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
