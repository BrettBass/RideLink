#ifndef GPS_HPP
#define GPS_HPP

#include <cstdint>

// Simple structure to hold location data
struct Location {
    double lat;
    double lon;
    bool valid;

    Location() : lat(0.0), lon(0.0), valid(false) {}
};

class GPS {
public:
    GPS();
    ~GPS();

    // Initialize GPS module on UART2 (default ESP32 pins: TX=GPIO17, RX=GPIO16)
    bool init(int rx_pin = 16, int tx_pin = 17, uint32_t baud = 9600);

    // Update GPS - call this regularly in your main loop
    // Returns true if new valid data was parsed
    bool update();

    // Get current location
    Location getLocation();

    // Check if we have a valid GPS fix
    bool isValid();

    // Get number of satellites
    uint8_t getSatellites();

    // Get speed in km/h
    float getSpeedKmh();

    // Get altitude in meters
    float getAltitudeMeters();

    // Calculate distance between two points (in meters)
    int calculateDistance(double lat1, double lon1, double lat2, double lon2);

    // Calculate bearing/angle from point 1 to point 2 (0-359 degrees, 0=North)
    int calculateBearing(double lat1, double lon1, double lat2, double lon2);

    // Calculate distance from current position to target (in meters)
    int distanceTo(double target_lat, double target_lon);

    // Calculate bearing from current position to target (0-359 degrees)
    int bearingTo(double target_lat, double target_lon);

    // Display GPS info to serial (for debugging)
    void displayInfo();

private:
    // NMEA parsing state
    char nmea_buffer[128];
    uint8_t nmea_index;
    bool parsing_sentence;

    // GPS data
    double current_lat;
    double current_lon;
    uint8_t satellites;
    float speed_kmh;
    float altitude_m;
    bool fix_valid;
    uint32_t last_update_ms;

    // Helper functions
    bool parseNMEA(const char* sentence);
    bool parseGPGGA(const char* sentence);
    bool parseGPRMC(const char* sentence);
    double parseCoordinate(const char* token, char hemisphere);
    uint8_t parseHex(char c);
    bool verifyChecksum(const char* sentence);

    // Convert degrees to radians
    double degreesToRadians(double degrees);
};

#endif // GPS_HPP

