# RideLink GPS Navigation System

## Overview

RideLink is a peer-to-peer GPS navigation system that enables two devices to locate and track each other using GPS, compass, and LoRa communication. Perfect for outdoor activities, hiking, biking, or any scenario where two people need to find each other without cellular connectivity.

## Features

- **Peer-to-Peer Tracking**: Two devices automatically find and track each other
- **GPS Navigation**: Real-time location tracking with distance calculation
- **Smart Compass**: Magnetometer with automatic calibration and hard/soft iron compensation
- **Long Range Communication**: LoRa radio for 1-2km range without infrastructure
- **Visual Indicators**: Color-coded arrow showing direction to peer
- **Persistent Calibration**: Compass calibration saved to non-volatile storage
- **Link Quality Monitoring**: RSSI and packet success rate tracking

## Hardware Requirements

### Core Components
- ESP32 Development Board (2x)
- NEO-6M GPS Module (2x)
- QMC5883L Compass Module (2x)
- SX1278 LoRa Module (2x)
- GC9A01 Round LCD Display (2x)

### Pin Connections

#### GPS Module (NEO-6M)
```
GPS TX  → ESP32 GPIO16 (RX2)
GPS RX  → ESP32 GPIO17 (TX2)
VCC     → 3.3V
GND     → GND
```

#### Compass Module (QMC5883L)
```
SDA → ESP32 GPIO21
SCL → ESP32 GPIO22
VCC → 3.3V
GND → GND
```

#### LoRa Module (SX1278)
```
SCK  → ESP32 GPIO18
MISO → ESP32 GPIO19
MOSI → ESP32 GPIO23
NSS  → ESP32 GPIO32
RST  → ESP32 GPIO12
DIO0 → ESP32 GPIO2
VCC  → 3.3V
GND  → GND
```

#### Display (GC9A01)
```
SCK  → ESP32 GPIO14
MOSI → ESP32 GPIO13
CS   → ESP32 GPIO15
DC   → ESP32 GPIO27
RST  → ESP32 GPIO26
BL   → ESP32 GPIO25
VCC  → 3.3V
GND  → GND
```

## Software Setup

### Prerequisites

1. Install ESP-IDF v5.0 or later
2. Set up ESP-IDF environment variables

### Building and Flashing

1. **Configure Device ID** (IMPORTANT!)
   
   Edit `RideLink.cpp` and set unique IDs:
   ```cpp
   #define DEVICE_ID 1  // Device 1
   ```
   ```cpp
   #define DEVICE_ID 2  // Device 2
   ```

2. **Set Frequency Region**
   ```cpp
   #define LORA_FREQUENCY 915.0  // US: 915.0, EU: 868.0, AS: 923.0
   ```

3. **Build the project**
   ```bash
   idf.py build
   ```

4. **Flash to ESP32**
   ```bash
   idf.py -p /dev/ttyUSB0 flash monitor
   ```

## Usage

### Display Indicators

#### Arrow Colors
- 🟢 **GREEN**: Pointing to peer device (both have GPS)
- 🟡 **YELLOW**: Peer found but no GPS fix
- 🔵 **CYAN**: Pointing north (GPS fix, no peer)
- ⚫ **GRAY**: Magnetic north only (no GPS, no peer)

#### Status Dots
- **Top-Right**: GPS status (green=fixed, red=searching)
- **Top-Center**: Peer connection (green=connected, red=searching)
- **Top-Left**: Calibration quality (green=good, yellow=fair, red=poor)

### Compass Calibration

1. **Enter Calibration Mode**
   - Hold BOOT button for 3 seconds

2. **Calibration Options**
   - **Quick Press**: Magnetometer calibration
   - **Hold 1 second**: Heading offset calibration
   - **Hold 5 seconds**: Clear all calibration

3. **Magnetometer Calibration Process**
   - Rotate device slowly in all directions
   - Make figure-8 patterns
   - Continue until progress bar completes
   - Calibration automatically saves to NVS

### Troubleshooting

#### No GPS Fix
- Ensure clear view of sky
- Wait 30-60 seconds for cold start
- Check antenna connection

#### No Peer Connection
- Verify both devices have different IDs
- Check LoRa antenna connections
- Ensure same frequency setting
- Reduce distance to <100m for initial test

#### Compass Issues
- Keep away from metal objects
- Perform calibration
- Check I2C connections

## Project Structure

```
RideLink/
├── main/
│   ├── RideLink.cpp         # Main application
│   └── CMakeLists.txt       # Component build config
├── include/
│   ├── GPS.hpp              # GPS interface
│   ├── Compass.hpp          # Compass interface
│   ├── Display.hpp          # Display interface
│   ├── LoRa.hpp            # LoRa interface
│   ├── CompassCalibrator.hpp # Calibration system
│   └── Config.hpp          # System configuration
├── src/
│   ├── GPS.cpp             # GPS implementation
│   ├── Compass.cpp         # Compass implementation
│   ├── Display.cpp         # Display implementation
│   ├── LoRa.cpp           # LoRa implementation
│   └── CompassCalibrator.cpp # Calibration implementation
├── CMakeLists.txt          # Project build config
└── README.md              # This file
```

## Performance Specifications

- **Update Rates**
  - GPS: 1 Hz
  - Compass: 10 Hz
  - Display: 10 Hz
  - LoRa broadcast: 0.5 Hz

- **Communication Range**
  - Urban: 500m - 1km
  - Open field: 1km - 2km
  - With elevation: 2km+

- **Power Consumption**
  - Active: ~200mA @ 3.3V
  - With GPS fix: ~150mA @ 3.3V

## Configuration Options

Edit `Config.hpp` to customize:

```cpp
// Timing
constexpr uint32_t LORA_BROADCAST_RATE_MS = 2000;  // Location broadcast rate
constexpr uint32_t PEER_TIMEOUT_MS = 10000;        // Connection timeout

// LoRa Parameters
constexpr uint8_t LORA_SPREADING_FACTOR = 9;  // 6-12 (higher=longer range)
constexpr uint32_t LORA_BANDWIDTH = 125000;    // Bandwidth in Hz
constexpr uint8_t LORA_TX_POWER = 17;          // 2-20 dBm

// Compass
constexpr float MAGNETIC_DECLINATION = 11.5;   // Local magnetic declination
```

## API Reference

### GPS Class
```cpp
bool isFixed()                    // Check if GPS has position fix
GPSPacket getPacket()             // Get compact GPS data for transmission
double getLatitude()              // Get current latitude
double getLongitude()             // Get current longitude
```

### Compass Class
```cpp
float getHeading()                // Get compass heading (0-359°)
void setDeclination(float decl)   // Set magnetic declination
void startCalibration()           // Begin calibration process
```

### LoRa Class
```cpp
bool sendPacket(const RideLinkPacket& packet)  // Send data packet
bool receivePacket(RideLinkPacket& packet)     // Receive data packet
int8_t getRSSI()                               // Get signal strength
const LinkStats& getStats()                    // Get link statistics
```

## Acknowledgments

- Coffee
