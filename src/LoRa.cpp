#include "LoRa.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cmath>

static const char* TAG = "LoRa";

LoRa::LoRa() : spi(nullptr), device_id(1), stats() {
}

LoRa::~LoRa() {
    if (spi != nullptr) {
        spi_bus_remove_device(spi);
    }
}

bool LoRa::begin(float frequency) {
    ESP_LOGI(TAG, "Initializing LoRa module at %.1f MHz", frequency);

    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = LORA_MOSI_PIN,
        .miso_io_num = LORA_MISO_PIN,
        .sclk_io_num = LORA_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256,
        .flags = 0,
        .intr_flags = 0
    };

    // Initialize SPI bus
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  // Bus might already be initialized
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return false;
    }

    // Configure SPI device
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,  // SPI mode 0
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 8000000,  // 8MHz
        .input_delay_ns = 0,
        .spics_io_num = LORA_NSS_PIN,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = nullptr,
        .post_cb = nullptr
    };

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return false;
    }

    // Configure GPIO pins
    gpio_config_t io_conf = {};

    // Reset pin
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LORA_RST_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // DIO0 pin (interrupt)
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << LORA_DIO0_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Reset the module
    reset();

    // Check version
    uint8_t version = readRegister(REG_VERSION);
    if (version != 0x12) {
        ESP_LOGE(TAG, "Invalid version: .x%02X (expected 0x12)", version);
        ESP_LOGE(TAG, "Check wiring: MISO=%d, MOSI=%d, SCK=%d, NSS=%d, RST=%d",
                 LORA_MISO_PIN, LORA_MOSI_PIN, LORA_SCK_PIN, LORA_NSS_PIN, LORA_RST_PIN);
        return false;
    }
    ESP_LOGI(TAG, "✓ SX1278 detected (version 0x%02X)", version);

    // Put in sleep mode
    sleep();

    // Set frequency
    setFrequency(frequency);

    // Set base addresses
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

    // Set LNA boost
    writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

    // Set auto AGC
    writeRegister(REG_MODEM_CONFIG_3, 0x04);

    // Set output power to 17 dBm
    setTxPower(17);

    // Put in standby mode
    idle();

    // Configure for default LoRa settings
    setSpreadingFactor(7);      // SF7 for good balance
    setBandwidth(125000);        // 125 kHz
    setCodingRate(5);           // 4/5
    setSyncWord(0x34);          // Private network
    enableCrc();

    ESP_LOGI(TAG, "✓ LoRa initialized successfully");
    ESP_LOGI(TAG, "  Frequency: %.1f MHz", frequency);
    ESP_LOGI(TAG, "  Spreading Factor: 7");
    ESP_LOGI(TAG, "  Bandwidth: 125 kHz");
    ESP_LOGI(TAG, "  TX Power: 17 dBm");

    return true;
}

void LoRa::reset() {
    gpio_set_level(LORA_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

uint8_t LoRa::singleTransfer(uint8_t address, uint8_t value) {
    uint8_t response;

    spi_transaction_t t = {};
    t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    t.length = 16;  // 2 bytes * 8 bits
    t.tx_data[0] = address;
    t.tx_data[1] = value;

    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transfer failed: %s", esp_err_to_name(ret));
        return 0;
    }

    response = t.rx_data[1];
    return response;
}

uint8_t LoRa::readRegister(uint8_t address) {
    return singleTransfer(address & 0x7F, 0x00);
}

void LoRa::writeRegister(uint8_t address, uint8_t value) {
    singleTransfer(address | 0x80, value);
}

void LoRa::setMode(uint8_t mode) {
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | mode);
}

void LoRa::setFrequency(float frequency) {
    uint64_t frf = ((uint64_t)frequency * 1000000) * 524288 / 32000000;

    writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void LoRa::setSpreadingFactor(uint8_t sf) {
    if (sf < 6) sf = 6;
    else if (sf > 12) sf = 12;

    if (sf == 6) {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }

    uint8_t config2 = readRegister(REG_MODEM_CONFIG_2);
    config2 = (config2 & 0x0F) | ((sf << 4) & 0xF0);
    writeRegister(REG_MODEM_CONFIG_2, config2);

    setLdoFlag();
}

void LoRa::setBandwidth(uint32_t bw) {
    uint8_t bw_val;

    if (bw <= 7800) bw_val = 0;
    else if (bw <= 10400) bw_val = 1;
    else if (bw <= 15600) bw_val = 2;
    else if (bw <= 20800) bw_val = 3;
    else if (bw <= 31250) bw_val = 4;
    else if (bw <= 41700) bw_val = 5;
    else if (bw <= 62500) bw_val = 6;
    else if (bw <= 125000) bw_val = 7;
    else if (bw <= 250000) bw_val = 8;
    else bw_val = 9;

    uint8_t config1 = readRegister(REG_MODEM_CONFIG_1);
    config1 = (config1 & 0x0F) | (bw_val << 4);
    writeRegister(REG_MODEM_CONFIG_1, config1);

    setLdoFlag();
}

void LoRa::setCodingRate(uint8_t cr) {
    if (cr < 5) cr = 5;
    else if (cr > 8) cr = 8;

    cr -= 4;

    uint8_t config1 = readRegister(REG_MODEM_CONFIG_1);
    config1 = (config1 & 0xF1) | (cr << 1);
    writeRegister(REG_MODEM_CONFIG_1, config1);
}

void LoRa::setTxPower(uint8_t level) {
    if (level > 20) level = 20;
    else if (level < 2) level = 2;

    // PA BOOST
    if (level > 17) {
        if (level > 20) {
            level = 20;
        }

        // subtract 3 from level, so 18 - 20 maps to 15 - 17
        level -= 3;

        // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
        writeRegister(REG_PA_DAC, 0x87);
        setOCP(140);
    } else {
        if (level < 2) {
            level = 2;
        }
        // Default value PA_HF/LF or +17dBm
        writeRegister(REG_PA_DAC, 0x84);
        setOCP(100);
    }

    writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

void LoRa::setSyncWord(uint8_t sw) {
    writeRegister(REG_SYNC_WORD, sw);
}

void LoRa::enableCrc() {
    uint8_t config2 = readRegister(REG_MODEM_CONFIG_2);
    writeRegister(REG_MODEM_CONFIG_2, config2 | 0x04);
}

void LoRa::disableCrc() {
    uint8_t config2 = readRegister(REG_MODEM_CONFIG_2);
    writeRegister(REG_MODEM_CONFIG_2, config2 & 0xFB);
}

void LoRa::setOCP(uint8_t mA) {
    uint8_t ocpTrim = 27;

    if (mA <= 120) {
        ocpTrim = (mA - 45) / 5;
    } else if (mA <= 240) {
        ocpTrim = (mA + 30) / 10;
    }

    writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void LoRa::setLdoFlag() {
    // Section 4.1.1.5
    uint8_t sf = (readRegister(REG_MODEM_CONFIG_2) >> 4);
    uint32_t bw = 125000;  // Default, would need to track actual BW

    // Symbol rate : Rs = BW / (2^SF)
    // Symbol duration: Ts = 1/Rs = (2^SF) / BW
    // Ts(msec) = (2^SF * 1000) / BW

    uint32_t symbolDuration = 1000 / (bw / (1L << sf));

    // Section 4.1.1.6
    bool ldoOn = symbolDuration > 16;

    uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
    bitWrite(config3, 3, ldoOn);
    writeRegister(REG_MODEM_CONFIG_3, config3);
}

void LoRa::sleep() {
    setMode(MODE_SLEEP);
}

void LoRa::idle() {
    setMode(MODE_STDBY);
}

void LoRa::receive() {
    setMode(MODE_RX_CONTINUOUS);
}

bool LoRa::sendBytes(const uint8_t* data, uint8_t length) {
    idle();

    // Reset FIFO address and payload length
    writeRegister(REG_FIFO_ADDR_PTR, 0);
    writeRegister(REG_PAYLOAD_LENGTH, 0);

    // Write data
    for (uint8_t i = 0; i < length; i++) {
        writeRegister(REG_FIFO, data[i]);
    }

    // Update length
    writeRegister(REG_PAYLOAD_LENGTH, length);

    // Start transmission
    setMode(MODE_TX);

    // Wait for TX done
    uint32_t timeout = esp_log_timestamp() + 2000;  // 2 second timeout
    while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
        if (esp_log_timestamp() > timeout) {
            ESP_LOGE(TAG, "TX timeout!");
            idle();
            stats.packets_failed++;
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Clear IRQ
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

    stats.packets_sent++;
    return true;
}

uint8_t LoRa::receiveBytes(uint8_t* data, uint8_t maxLength) {
    uint8_t irqFlags = readRegister(REG_IRQ_FLAGS);

    // Clear IRQ
    writeRegister(REG_IRQ_FLAGS, irqFlags);

    if ((irqFlags & IRQ_RX_DONE_MASK) == 0) {
        return 0;  // No packet received
    }

    if (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) {
        ESP_LOGW(TAG, "CRC error!");
        stats.packets_failed++;
        return 0;
    }

    // Read packet length
    uint8_t length = readRegister(REG_RX_NB_BYTES);
    if (length > maxLength) length = maxLength;

    // Read FIFO
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    for (uint8_t i = 0; i < length; i++) {
        data[i] = readRegister(REG_FIFO);
    }

    stats.packets_received++;
    stats.last_rx_time = esp_log_timestamp();
    stats.last_rssi = getPacketRSSI();
    stats.last_snr = getPacketSNR();

    return length;
}

bool LoRa::sendPacket(const RideLinkPacket& packet) {
    RideLinkPacket txPacket = packet;
    txPacket.device_id = device_id;
    txPacket.timestamp = esp_log_timestamp();
    txPacket.checksum = txPacket.calculateChecksum();

    return sendBytes((const uint8_t*)&txPacket, sizeof(RideLinkPacket));
}

bool LoRa::receivePacket(RideLinkPacket& packet, uint32_t timeout_ms) {
    if (timeout_ms > 0) {
        if (!waitForPacket(timeout_ms)) {
            return false;
        }
    }

    uint8_t buffer[sizeof(RideLinkPacket)];
    uint8_t length = receiveBytes(buffer, sizeof(buffer));

    if (length != sizeof(RideLinkPacket)) {
        if (length > 0) {
            ESP_LOGW(TAG, "Invalid packet size: %d (expected %d)", length, sizeof(RideLinkPacket));
        }
        return false;
    }

    memcpy(&packet, buffer, sizeof(RideLinkPacket));

    if (!packet.verifyChecksum()) {
        ESP_LOGW(TAG, "Packet checksum failed!");
        stats.packets_failed++;
        return false;
    }

    packet.rssi = stats.last_rssi;

    return true;
}

bool LoRa::broadcastLocation(const GPSPacket& gps, int16_t heading) {
    RideLinkPacket packet;
    packet.packet_type = 0x01;  // Location broadcast
    packet.device_id = device_id;
    packet.gps = gps;
    packet.compass_heading = heading;
    packet.battery_level = 100;  // TODO: Add real battery monitoring

    return sendPacket(packet);
}

bool LoRa::available() {
    return (readRegister(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) != 0;
}

bool LoRa::waitForPacket(uint32_t timeout_ms) {
    uint32_t start = esp_log_timestamp();

    while (!available()) {
        if (esp_log_timestamp() - start > timeout_ms) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return true;
}

int8_t LoRa::getRSSI() {
    return readRegister(REG_RSSI_VALUE) - 164;
}

int8_t LoRa::getSNR() {
    return (int8_t)readRegister(REG_PKT_SNR_VALUE) / 4;
}

int8_t LoRa::getPacketRSSI() {
    int8_t snr = getSNR();
    int16_t rssi = readRegister(REG_PKT_RSSI_VALUE);

    // Adjust for low data rate optimization
    if (snr < 0) {
        rssi = rssi - 164 + snr;
    } else {
        rssi = rssi - 164;
    }

    return rssi;
}

int8_t LoRa::getPacketSNR() {
    return getSNR();
}

void LoRa::clearIRQ() {
    writeRegister(REG_IRQ_FLAGS, 0xFF);
}

void LoRa::printRegisters() {
    ESP_LOGI(TAG, "=== LoRa Registers ===");
    ESP_LOGI(TAG, "OP_MODE: 0x%02X", readRegister(REG_OP_MODE));
    ESP_LOGI(TAG, "MODEM_CONFIG_1: 0x%02X", readRegister(REG_MODEM_CONFIG_1));
    ESP_LOGI(TAG, "MODEM_CONFIG_2: 0x%02X", readRegister(REG_MODEM_CONFIG_2));
    ESP_LOGI(TAG, "MODEM_CONFIG_3: 0x%02X", readRegister(REG_MODEM_CONFIG_3));
    ESP_LOGI(TAG, "PA_CONFIG: 0x%02X", readRegister(REG_PA_CONFIG));
    ESP_LOGI(TAG, "IRQ_FLAGS: 0x%02X", readRegister(REG_IRQ_FLAGS));
    ESP_LOGI(TAG, "RSSI: %d dBm", getRSSI());
}

void LoRa::printStatus() {
    ESP_LOGI(TAG, "=== LoRa Link Stats ===");
    ESP_LOGI(TAG, "Device ID: %d", device_id);
    ESP_LOGI(TAG, "Packets Sent: %lu", (unsigned long)stats.packets_sent);
    ESP_LOGI(TAG, "Packets Received: %lu", (unsigned long)stats.packets_received);
    ESP_LOGI(TAG, "Packets Failed: %lu", (unsigned long)stats.packets_failed);
    ESP_LOGI(TAG, "Last RSSI: %d dBm", stats.last_rssi);
    ESP_LOGI(TAG, "Last SNR: %d dB", stats.last_snr);
    ESP_LOGI(TAG, "Current RSSI: %d dBm", getRSSI());
}

// Helper for bit manipulation
void LoRa::bitWrite(uint8_t& value, uint8_t bit, uint8_t bitValue) {
    if (bitValue) {
        value |= (1 << bit);
    } else {
        value &= ~(1 << bit);
    }
}
