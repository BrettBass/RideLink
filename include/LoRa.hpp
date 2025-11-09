#ifndef LORA_HPP
#define LORA_HPP

#include <cstdint>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "GPS.hpp"

// SX1278 LoRa module pin definitions
#define LORA_RST_PIN    GPIO_NUM_12
#define LORA_DIO0_PIN   GPIO_NUM_2
#define LORA_NSS_PIN    GPIO_NUM_32
#define LORA_SCK_PIN    GPIO_NUM_18
#define LORA_MISO_PIN   GPIO_NUM_19
#define LORA_MOSI_PIN   GPIO_NUM_23

// SX1278 Register addresses
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0B
#define REG_LNA                  0x0C
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_RSSI_VALUE           0x1B
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2A
#define REG_RSSI_WIDEBAND        0x2C
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3B
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4D

// Modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

// Packet structure for RideLink communication
// Pack the structure to avoid padding issues
#pragma pack(push, 1)
struct RideLinkPacket {
    uint8_t packet_type;     // 0x01 = location broadcast, 0x02 = ack
    uint8_t device_id;       // Unique device identifier
    GPSPacket gps;           // 12 bytes of GPS data
    int16_t compass_heading; // Current compass heading
    uint8_t battery_level;   // Battery percentage
    uint32_t timestamp;      // Millisecond timestamp
    int8_t rssi;            // Signal strength of last received packet
    uint8_t checksum;       // Simple checksum for data integrity

    RideLinkPacket() : packet_type(0x01), device_id(0), compass_heading(0),
                       battery_level(100), timestamp(0), rssi(0), checksum(0) {}

    uint8_t calculateChecksum() const {
        const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
        uint8_t sum = 0;
        // Calculate checksum for all bytes except the checksum field itself
        for (size_t i = 0; i < sizeof(RideLinkPacket) - 1; i++) {
            sum += data[i];  // Changed from XOR to addition for better detection
        }
        return sum;
    }

    bool verifyChecksum() const {
        uint8_t calculated = calculateChecksum();
        return checksum == calculated;
    }
};
#pragma pack(pop)

// Statistics for link quality
struct LinkStats {
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t packets_failed;
    int8_t last_rssi;
    int8_t last_snr;
    uint32_t last_rx_time;

    LinkStats() : packets_sent(0), packets_received(0), packets_failed(0),
                  last_rssi(-120), last_snr(0), last_rx_time(0) {}
};

class LoRa {
public:
    LoRa();
    ~LoRa();

    // Initialize LoRa module with frequency (MHz)
    bool begin(float frequency = 915.0);

    // Configuration
    void setSpreadingFactor(uint8_t sf);     // 6-12
    void setBandwidth(uint32_t bw);          // Hz
    void setCodingRate(uint8_t cr);          // 5-8
    void setTxPower(uint8_t power);          // 2-20 dBm
    void setSyncWord(uint8_t sw);            // Network ID
    void enableCrc();
    void disableCrc();

    // Set device ID for this unit
    void setDeviceId(uint8_t id) { device_id = id; }
    uint8_t getDeviceId() const { return device_id; }

    // Send/receive RideLink packets
    bool sendPacket(const RideLinkPacket& packet);
    bool receivePacket(RideLinkPacket& packet, uint32_t timeout_ms = 0);

    // Send location broadcast
    bool broadcastLocation(const GPSPacket& gps, int16_t heading);

    // Check if packet is available
    bool available();

    // Get signal strength and SNR
    int8_t getRSSI();
    int8_t getSNR();
    int8_t getPacketRSSI();
    int8_t getPacketSNR();

    // Get link statistics
    const LinkStats& getStats() const { return stats; }
    void resetStats() { stats = LinkStats(); }

    // Low-level operations
    void sleep();
    void idle();
    void receive();

    // Low-level packet handling (public for testing)
    bool sendBytes(const uint8_t* data, uint8_t length);
    uint8_t receiveBytes(uint8_t* data, uint8_t maxLength);

    // Debug
    void printRegisters();
    void printStatus();

private:
    spi_device_handle_t spi;
    uint8_t device_id;
    LinkStats stats;

    // SPI communication
    uint8_t readRegister(uint8_t address);
    void writeRegister(uint8_t address, uint8_t value);
    uint8_t singleTransfer(uint8_t address, uint8_t value);

    // Module control
    void reset();
    void setMode(uint8_t mode);
    void setFrequency(float frequency);
    void setOCP(uint8_t mA);
    void setLdoFlag();

    // Interrupt handling
    void clearIRQ();
    bool waitForPacket(uint32_t timeout_ms);

    // Helper functions
    void bitWrite(uint8_t& value, uint8_t bit, uint8_t bitValue);
};

#endif // LORA_HPP
