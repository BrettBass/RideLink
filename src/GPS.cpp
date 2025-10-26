#include "GPS.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include <cstring>
#include <cmath>
#include <cstdlib>

static const char *TAG = "GPS";

#define UART_NUM UART_NUM_2
#define UART_BUF_SIZE 1024
#define EARTH_RADIUS_KM 6371.0

GPS::GPS()
    : nmea_index(0)
    , parsing_sentence(false)
    , current_lat(0.0)
    , current_lon(0.0)
    , satellites(0)
    , speed_kmh(0.0f)
    , altitude_m(0.0f)
    , fix_valid(false)
    , last_update_ms(0)
{
    memset(nmea_buffer, 0, sizeof(nmea_buffer));
}

GPS::~GPS()
{
    uart_driver_delete(UART_NUM);
}

bool GPS::init(int rx_pin, int tx_pin, uint32_t baud)
{
    ESP_LOGI(TAG, "Initializing GPS on UART2 (RX=GPIO%d, TX=GPIO%d, baud=%lu)",
             rx_pin, tx_pin, baud);

    uart_config_t uart_config = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    // Configure UART parameters
    esp_err_t ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        return false;
    }

    // Set UART pins
    ret = uart_set_pin(UART_NUM, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return false;
    }

    // Install UART driver
    ret = uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "GPS initialized successfully");
    ESP_LOGI(TAG, "Waiting for GPS fix... (may take 30-60 seconds outdoors)");

    return true;
}

bool GPS::update()
{
    uint8_t data[128];
    int length = uart_read_bytes(UART_NUM, data, sizeof(data) - 1, 10 / portTICK_PERIOD_MS);

    if (length > 0) {
        data[length] = '\0';  // Null terminate

        // Process each byte
        for (int i = 0; i < length; i++) {
            char c = (char)data[i];

            // Look for start of NMEA sentence
            if (c == '$') {
                nmea_index = 0;
                parsing_sentence = true;
                nmea_buffer[nmea_index++] = c;
            }
            // End of NMEA sentence
            else if (c == '\n' || c == '\r') {
                if (parsing_sentence && nmea_index > 0) {
                    nmea_buffer[nmea_index] = '\0';

                    // Parse the complete sentence
                    if (parseNMEA(nmea_buffer)) {
                        last_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                        parsing_sentence = false;
                        return true;  // New valid data
                    }
                    parsing_sentence = false;
                }
                nmea_index = 0;
            }
            // Add to buffer
            else if (parsing_sentence && nmea_index < sizeof(nmea_buffer) - 1) {
                nmea_buffer[nmea_index++] = c;
            }
        }
    }

    return false;
}

bool GPS::parseNMEA(const char* sentence)
{
    // Verify checksum
    if (!verifyChecksum(sentence)) {
        return false;
    }

    // Parse different sentence types
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        return parseGPGGA(sentence);
    }
    else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        return parseGPRMC(sentence);
    }

    return false;
}

bool GPS::parseGPGGA(const char* sentence)
{
    // $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
    // Example: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47

    char buffer[128];
    strncpy(buffer, sentence, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char* token = strtok(buffer, ",");
    int field = 0;

    char lat_str[16] = {0};
    char lat_hem = 'N';
    char lon_str[16] = {0};
    char lon_hem = 'E';
    int quality = 0;

    while (token != NULL) {
        switch (field) {
            case 2: // Latitude
                strncpy(lat_str, token, sizeof(lat_str) - 1);
                break;
            case 3: // N/S
                lat_hem = token[0];
                break;
            case 4: // Longitude
                strncpy(lon_str, token, sizeof(lon_str) - 1);
                break;
            case 5: // E/W
                lon_hem = token[0];
                break;
            case 6: // Fix quality (0=invalid, 1=GPS, 2=DGPS)
                quality = atoi(token);
                break;
            case 7: // Number of satellites
                satellites = atoi(token);
                break;
            case 9: // Altitude
                altitude_m = atof(token);
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    if (quality > 0 && strlen(lat_str) > 0 && strlen(lon_str) > 0) {
        current_lat = parseCoordinate(lat_str, lat_hem);
        current_lon = parseCoordinate(lon_str, lon_hem);
        fix_valid = true;
        return true;
    }

    fix_valid = false;
    return false;
}

bool GPS::parseGPRMC(const char* sentence)
{
    // $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
    // Example: $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A

    char buffer[128];
    strncpy(buffer, sentence, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char* token = strtok(buffer, ",");
    int field = 0;

    char status = 'V';
    char lat_str[16] = {0};
    char lat_hem = 'N';
    char lon_str[16] = {0};
    char lon_hem = 'E';

    while (token != NULL) {
        switch (field) {
            case 2: // Status (A=active, V=void)
                status = token[0];
                break;
            case 3: // Latitude
                strncpy(lat_str, token, sizeof(lat_str) - 1);
                break;
            case 4: // N/S
                lat_hem = token[0];
                break;
            case 5: // Longitude
                strncpy(lon_str, token, sizeof(lon_str) - 1);
                break;
            case 6: // E/W
                lon_hem = token[0];
                break;
            case 7: // Speed in knots
                speed_kmh = atof(token) * 1.852f;  // Convert knots to km/h
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    if (status == 'A' && strlen(lat_str) > 0 && strlen(lon_str) > 0) {
        current_lat = parseCoordinate(lat_str, lat_hem);
        current_lon = parseCoordinate(lon_str, lon_hem);
        fix_valid = true;
        return true;
    }

    return false;
}

double GPS::parseCoordinate(const char* token, char hemisphere)
{
    if (strlen(token) == 0) return 0.0;

    // Format: ddmm.mmmm or dddmm.mmmm
    double coord = atof(token);

    // Extract degrees (everything before last 2 digits before decimal)
    int degrees = (int)(coord / 100);

    // Extract minutes (last 2 digits before decimal + everything after)
    double minutes = coord - (degrees * 100);

    // Convert to decimal degrees
    double decimal = degrees + (minutes / 60.0);

    // Apply hemisphere
    if (hemisphere == 'S' || hemisphere == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

bool GPS::verifyChecksum(const char* sentence)
{
    if (sentence[0] != '$') return false;

    // Find the asterisk
    const char* asterisk = strchr(sentence, '*');
    if (!asterisk || asterisk == sentence) return false;

    // Calculate checksum (XOR of all characters between $ and *)
    uint8_t checksum = 0;
    for (const char* p = sentence + 1; p < asterisk; p++) {
        checksum ^= *p;
    }

    // Parse the checksum from the sentence
    uint8_t expected = 0;
    if (strlen(asterisk) >= 3) {
        expected = (parseHex(asterisk[1]) << 4) | parseHex(asterisk[2]);
    }

    return checksum == expected;
}

uint8_t GPS::parseHex(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

Location GPS::getLocation()
{
    Location loc;
    loc.lat = current_lat;
    loc.lon = current_lon;
    loc.valid = fix_valid;
    return loc;
}

bool GPS::isValid()
{
    return fix_valid;
}

uint8_t GPS::getSatellites()
{
    return satellites;
}

float GPS::getSpeedKmh()
{
    return speed_kmh;
}

float GPS::getAltitudeMeters()
{
    return altitude_m;
}

double GPS::degreesToRadians(double degrees)
{
    return degrees * M_PI / 180.0;
}

int GPS::calculateDistance(double lat1, double lon1, double lat2, double lon2)
{
    // Haversine formula
    double lat1_rad = degreesToRadians(lat1);
    double lon1_rad = degreesToRadians(lon1);
    double lat2_rad = degreesToRadians(lat2);
    double lon2_rad = degreesToRadians(lon2);

    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;

    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dlon / 2.0) * sin(dlon / 2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    // Distance in meters
    double distance = EARTH_RADIUS_KM * c * 1000.0;

    return (int)distance;
}

int GPS::calculateBearing(double lat1, double lon1, double lat2, double lon2)
{
    // Calculate bearing from point 1 to point 2
    double lat1_rad = degreesToRadians(lat1);
    double lon1_rad = degreesToRadians(lon1);
    double lat2_rad = degreesToRadians(lat2);
    double lon2_rad = degreesToRadians(lon2);

    double dlon = lon2_rad - lon1_rad;

    double y = sin(dlon) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) -
               sin(lat1_rad) * cos(lat2_rad) * cos(dlon);

    double bearing = atan2(y, x);

    // Convert to degrees
    bearing = bearing * 180.0 / M_PI;

    // Normalize to 0-359
    bearing = fmod(bearing + 360.0, 360.0);

    return (int)bearing;
}

int GPS::distanceTo(double target_lat, double target_lon)
{
    if (!fix_valid) return -1;
    return calculateDistance(current_lat, current_lon, target_lat, target_lon);
}

int GPS::bearingTo(double target_lat, double target_lon)
{
    if (!fix_valid) return -1;
    return calculateBearing(current_lat, current_lon, target_lat, target_lon);
}

void GPS::displayInfo()
{
    if (fix_valid) {
        ESP_LOGI(TAG, "Location: %.6f, %.6f | Sats: %d | Speed: %.1f km/h | Alt: %.1f m",
                current_lat, current_lon, satellites, speed_kmh, altitude_m);
    } else {
        ESP_LOGI(TAG, "GPS: No fix yet (Satellites: %d)", satellites);
    }
}

