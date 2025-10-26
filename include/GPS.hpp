#ifndef GPS_HPP
#define GPS_HPP

#include <cstdint>

// Compact GPS packet for LoRa transmission
struct GPSPacket {
    float lat;           // 4 bytes
    float lon;           // 4 bytes
    uint16_t altitude;   // 2 bytes (meters, 0-65535m range)
    uint8_t satellites;  // 1 byte
    uint8_t hdop;        // 1 byte (scaled by 10, so 2.5 = 25)
    // Total: 12 bytes - efficient for LoRa

    GPSPacket() : lat(0), lon(0), altitude(0), satellites(0), hdop(255) {}
};

class GPS {
public:
    GPS();
    ~GPS();

    // Initialize GPS module on UART2 (TX2/RX2 pins)
    bool begin();

    // Read and process GPS data
    void read();

    // Check if position is valid
    bool isFixed() const { return is_fixed; }

    // Get compact packet for transmission
    GPSPacket getPacket() const;

    // Get raw values
    double getLatitude() const { return latitude; }
    double getLongitude() const { return longitude; }
    uint8_t getSatelliteCount() const { return satellite_count; }
    float getHDOP() const { return hdop_value; }

    // Calculate distance to another GPS point (meters)
    static uint32_t calculateDistance(double lat1, double lon1, double lat2, double lon2);

    // Calculate compass heading to another point (0-359)
    static uint16_t calculateHeading(double lat1, double lon1, double lat2, double lon2);

    // Debug output
    void printStatus() const;

private:
    // GPS state
    double latitude;
    double longitude;
    float altitude;
    float hdop_value;
    uint8_t satellite_count;
    bool is_fixed;
    uint32_t last_fix_time;

    // NMEA parsing
    char nmea_line[100];
    uint8_t line_index;

    void processChar(char c);
    void processLine();
    bool parseGGA(const char* line);
    bool parseRMC(const char* line);
    double convertNMEAtoDecimal(const char* coord, char dir);
};

#endif // GPS_HPP

