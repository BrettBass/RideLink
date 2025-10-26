#include "GPS.hpp"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include <cstring>
#include <cstdlib>
#include <cmath>

static const char* TAG = "GPS";

// ESP32 UART2 pins - can be any GPIO
#define GPS_TX_PIN GPIO_NUM_26  // TX2 (GPS RX connects here)
#define GPS_RX_PIN GPIO_NUM_25  // RX2 (GPS TX connects here)
#define GPS_UART_NUM UART_NUM_2
#define GPS_BAUD_RATE 9600

GPS::GPS()
    : latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , hdop_value(99.9)
    , satellite_count(0)
    , is_fixed(false)
    , last_fix_time(0)
    , line_index(0) {
    memset(nmea_line, 0, sizeof(nmea_line));
}

GPS::~GPS() {
    uart_driver_delete(GPS_UART_NUM);
}

bool GPS::begin() {
    ESP_LOGI(TAG, "Initializing GPS on TX=%d (to GPS RX), RX=%d (to GPS TX)", GPS_TX_PIN, GPS_RX_PIN);

    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Configure UART
    if (uart_param_config(GPS_UART_NUM, &uart_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART");
        return false;
    }

    // Set pins
    if (uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins");
        return false;
    }

    // Install driver
    if (uart_driver_install(GPS_UART_NUM, 256, 0, 0, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver");
        return false;
    }

    ESP_LOGI(TAG, "GPS initialized successfully");
    return true;
}

void GPS::read() {
    uint8_t data;
    int len = uart_read_bytes(GPS_UART_NUM, &data, 1, 0);

    if (len > 0) {
        processChar(data);
    }
}

void GPS::processChar(char c) {
    if (c == '$') {
        // Start of new sentence
        line_index = 0;
        nmea_line[line_index++] = c;
    }
    else if (c == '\n' || c == '\r') {
        // End of sentence
        if (line_index > 0) {
            nmea_line[line_index] = '\0';
            processLine();
            line_index = 0;
        }
    }
    else if (line_index < sizeof(nmea_line) - 1) {
        // Add to buffer
        nmea_line[line_index++] = c;
    }
}

void GPS::processLine() {
    // Check for GGA (position data) or RMC (minimum data)
    if (strncmp(nmea_line, "$GNGGA", 6) == 0 || strncmp(nmea_line, "$GPGGA", 6) == 0) {
        parseGGA(nmea_line);
    }
    else if (strncmp(nmea_line, "$GNRMC", 6) == 0 || strncmp(nmea_line, "$GPRMC", 6) == 0) {
        parseRMC(nmea_line);
    }
    // We can ignore GSV (satellites in view) for now - GGA gives us count
}

bool GPS::parseGGA(const char* line) {
    // $GNGGA,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,,,,,*checksum
    char temp[100];
    strncpy(temp, line, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';

    char* token = strtok(temp, ",");
    int field = 0;

    double new_lat = 0, new_lon = 0;
    char lat_dir = 0, lon_dir = 0;
    int fix_quality = 0;

    while (token != NULL && field < 15) {
        switch (field) {
            case 2: // Latitude
                if (strlen(token) > 0) {
                    new_lat = atof(token);
                }
                break;
            case 3: // N/S
                if (strlen(token) > 0) {
                    lat_dir = token[0];
                }
                break;
            case 4: // Longitude
                if (strlen(token) > 0) {
                    new_lon = atof(token);
                }
                break;
            case 5: // E/W
                if (strlen(token) > 0) {
                    lon_dir = token[0];
                }
                break;
            case 6: // Fix quality (0=no fix, 1=GPS, 2=DGPS)
                fix_quality = atoi(token);
                break;
            case 7: // Number of satellites
                satellite_count = atoi(token);
                break;
            case 8: // HDOP
                if (strlen(token) > 0) {
                    hdop_value = atof(token);
                }
                break;
            case 9: // Altitude
                if (strlen(token) > 0) {
                    altitude = atof(token);
                }
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    // Update position if we have a fix
    if (fix_quality > 0 && new_lat > 0 && new_lon > 0) {
        latitude = convertNMEAtoDecimal(temp + 7, lat_dir);  // Rough position in string
        longitude = convertNMEAtoDecimal(temp + 20, lon_dir); // Rough position

        // Actually, let's reparse more carefully
        char lat_str[20], lon_str[20];
        sscanf(line, "$%*[^,],%*[^,],%[^,],%c,%[^,],%c",
               lat_str, &lat_dir, lon_str, &lon_dir);

        if (strlen(lat_str) > 0 && strlen(lon_str) > 0) {
            latitude = convertNMEAtoDecimal(lat_str, lat_dir);
            longitude = convertNMEAtoDecimal(lon_str, lon_dir);
            is_fixed = true;
            last_fix_time = esp_log_timestamp();
        }
    } else {
        is_fixed = false;
    }

    return is_fixed;
}

bool GPS::parseRMC(const char* line) {
    // $GNRMC,time,status,lat,N/S,lon,E/W,speed,course,date,,,mode*checksum
    char temp[100];
    strncpy(temp, line, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';

    char* token = strtok(temp, ",");
    int field = 0;

    char status = 'V';
    double new_lat = 0, new_lon = 0;
    char lat_dir = 0, lon_dir = 0;

    while (token != NULL && field < 12) {
        switch (field) {
            case 2: // Status (A=active/valid, V=void/invalid)
                if (strlen(token) > 0) {
                    status = token[0];
                }
                break;
            case 3: // Latitude
                if (strlen(token) > 0) {
                    new_lat = atof(token);
                }
                break;
            case 4: // N/S
                if (strlen(token) > 0) {
                    lat_dir = token[0];
                }
                break;
            case 5: // Longitude
                if (strlen(token) > 0) {
                    new_lon = atof(token);
                }
                break;
            case 6: // E/W
                if (strlen(token) > 0) {
                    lon_dir = token[0];
                }
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    // Update if valid
    if (status == 'A' && new_lat > 0 && new_lon > 0) {
        // Re-parse for accurate values
        char lat_str[20], lon_str[20];
        sscanf(line, "$%*[^,],%*[^,],%*[^,],%[^,],%c,%[^,],%c",
               lat_str, &lat_dir, lon_str, &lon_dir);

        if (strlen(lat_str) > 0 && strlen(lon_str) > 0) {
            latitude = convertNMEAtoDecimal(lat_str, lat_dir);
            longitude = convertNMEAtoDecimal(lon_str, lon_dir);
            is_fixed = true;
            last_fix_time = esp_log_timestamp();
        }
    }

    return is_fixed;
}

double GPS::convertNMEAtoDecimal(const char* coord, char dir) {
    // NMEA format: ddmm.mmmm (latitude) or dddmm.mmmm (longitude)
    double value = atof(coord);

    // Extract degrees
    int degrees = (int)(value / 100);

    // Extract minutes
    double minutes = value - (degrees * 100);

    // Convert to decimal
    double decimal = degrees + (minutes / 60.0);

    // Apply direction
    if (dir == 'S' || dir == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

GPSPacket GPS::getPacket() const {
    GPSPacket packet;
    packet.lat = (float)latitude;
    packet.lon = (float)longitude;
    packet.altitude = (uint16_t)altitude;
    packet.satellites = satellite_count;
    packet.hdop = (uint8_t)(hdop_value * 10); // Scale to fit in byte
    return packet;
}

uint32_t GPS::calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    // Haversine formula
    const double R = 6371000; // Earth radius in meters

    double lat1_rad = lat1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double delta_lat = (lat2 - lat1) * M_PI / 180.0;
    double delta_lon = (lon2 - lon1) * M_PI / 180.0;

    double a = sin(delta_lat/2) * sin(delta_lat/2) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(delta_lon/2) * sin(delta_lon/2);

    double c = 2 * atan2(sqrt(a), sqrt(1-a));

    return (uint32_t)(R * c);
}

uint16_t GPS::calculateHeading(double lat1, double lon1, double lat2, double lon2) {
    double lat1_rad = lat1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double delta_lon = (lon2 - lon1) * M_PI / 180.0;

    double y = sin(delta_lon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) -
               sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon);

    double heading = atan2(y, x) * 180.0 / M_PI;

    // Normalize to 0-359
    heading = fmod(heading + 360.0, 360.0);

    return (uint16_t)heading;
}

void GPS::printStatus() const {
    if (is_fixed) {
        ESP_LOGI(TAG, "GPS FIX | Lat: %.6f | Lon: %.6f | Sats: %d | HDOP: %.1f | Alt: %.0fm",
                 latitude, longitude, satellite_count, hdop_value, altitude);
    } else {
        ESP_LOGI(TAG, "NO FIX | Satellites: %d | HDOP: %.1f",
                 satellite_count, hdop_value);
    }
}

