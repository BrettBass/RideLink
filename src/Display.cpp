#include "Display.hpp"
#include "esp_lcd_gc9a01.h"
#include "esp_log.h"
#include "soc/gpio_num.h"
#include <cstring>
#include <cmath>
#include <algorithm>

static const char *TAG = "DISPLAY";

// Display Configuration
#define LCD_HOST      SPI3_HOST
#define LCD_RST_GPIO  GPIO_NUM_33
#define LCD_CS_GPIO   GPIO_NUM_5
#define LCD_DC_GPIO   GPIO_NUM_27
#define LCD_MOSI_GPIO GPIO_NUM_15
#define LCD_CLK_GPIO  GPIO_NUM_14

#define LCD_H_RES 240
#define LCD_V_RES 240
#define LCD_PIXEL_CLOCK_HZ (10 * 1000 * 1000)

Display::Display()
    : panel_handle(nullptr)
    , io_handle(nullptr)
    , spi_handle(nullptr)
    , framebuffer(nullptr)
    , width(LCD_H_RES)
    , height(LCD_V_RES)
    , lastArrowAngle(0)
    , hasLastArrow(false)
{
}

Display::~Display()
{
    if (framebuffer) {
        heap_caps_free(framebuffer);
    }
}

bool Display::init()
{
    ESP_LOGI(TAG, "Initializing GC9A01 display");

    // Allocate framebuffer for row operations
    framebuffer = (uint16_t *)heap_caps_malloc(width * 2, MALLOC_CAP_DMA);
    if (!framebuffer) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer");
        return false;
    }

    // Initialize SPI Bus
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = LCD_MOSI_GPIO;
    bus_config.miso_io_num = GPIO_NUM_NC;
    bus_config.sclk_io_num = LCD_CLK_GPIO;
    bus_config.quadwp_io_num = GPIO_NUM_NC;
    bus_config.quadhd_io_num = GPIO_NUM_NC;
    bus_config.data4_io_num = GPIO_NUM_NC;
    bus_config.data5_io_num = GPIO_NUM_NC;
    bus_config.data6_io_num = GPIO_NUM_NC;
    bus_config.data7_io_num = GPIO_NUM_NC;
    bus_config.max_transfer_sz = width * height * 2 + 8;
    bus_config.flags = SPICOMMON_BUSFLAG_MASTER;
    bus_config.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;
    bus_config.intr_flags = 0;

    esp_err_t ret = spi_bus_initialize(LCD_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return false;
    }

    // Initialize Panel IO
    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.cs_gpio_num = LCD_CS_GPIO;
    io_config.dc_gpio_num = LCD_DC_GPIO;
    io_config.spi_mode = 0;
    io_config.pclk_hz = LCD_PIXEL_CLOCK_HZ;
    io_config.trans_queue_depth = 10;
    io_config.on_color_trans_done = nullptr;
    io_config.user_ctx = nullptr;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.flags.dc_low_on_data = 0;
    io_config.flags.octal_mode = 0;
    io_config.flags.lsb_first = 0;

    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create panel IO");
        return false;
    }

    // Install GC9A01 Panel Driver
    const gc9a01_vendor_config_t vendor_config = {
        .init_cmds = nullptr,
        .init_cmds_size = 0
    };

    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = LCD_RST_GPIO;
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_config.bits_per_pixel = 16;
    panel_config.flags.reset_active_high = 0;
    panel_config.vendor_config = (void *)&vendor_config;

    ret = esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create panel driver");
        return false;
    }

    // Reset and initialize
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);

    // Invert colors - many GC9A01 displays need this
    esp_lcd_panel_invert_color(panel_handle, true);
    esp_lcd_panel_mirror(panel_handle, false, false);
    esp_lcd_panel_disp_on_off(panel_handle, true);

    ESP_LOGI(TAG, "Display initialized successfully");
    return true;
}

void Display::fillScreen(uint16_t color)
{
    // Create a larger buffer for better performance
    const int bufferSize = width * 10; // 10 rows at a time
    uint16_t *buffer = (uint16_t *)heap_caps_malloc(bufferSize * 2, MALLOC_CAP_DMA);

    if (!buffer) {
        // Fallback to single row buffer
        for (int i = 0; i < width; i++) {
            framebuffer[i] = color;
        }
        for (int y = 0; y < height; y++) {
            esp_lcd_panel_draw_bitmap(panel_handle, 0, y, width, y + 1, framebuffer);
        }
        return;
    }

    // Fill buffer with color
    for (int i = 0; i < bufferSize; i++) {
        buffer[i] = color;
    }

    // Draw 10 rows at a time
    for (int y = 0; y < height; y += 10) {
        int rows = (y + 10 <= height) ? 10 : (height - y);
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, width, y + rows, buffer);
    }

    heap_caps_free(buffer);
}

void Display::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    if (x >= width || y >= height) return;
    if (x + w > width) w = width - x;
    if (y + h > height) h = height - y;

    // Fill buffer with color
    for (int i = 0; i < w; i++) {
        framebuffer[i] = color;
    }

    // Draw each row of the rectangle
    for (int row = 0; row < h; row++) {
        esp_lcd_panel_draw_bitmap(panel_handle, x, y + row, x + w, y + row + 1, framebuffer);
    }
}

void Display::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    if (x < 0 || x >= width || y < 0 || y >= height) return;
    esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + 1, y + 1, &color);
}

void Display::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        swapInt16(x0, y0);
        swapInt16(x1, y1);
    }

    if (x0 > x1) {
        swapInt16(x0, x1);
        swapInt16(y0, y1);
    }

    int16_t dx = x1 - x0;
    int16_t dy = abs(y1 - y0);
    int16_t err = dx / 2;
    int16_t ystep = (y0 < y1) ? 1 : -1;

    for (; x0 <= x1; x0++) {
        if (steep) {
            drawPixel(y0, x0, color);
        } else {
            drawPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

void Display::drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    drawLine(x0, y0, x1, y1, color);
    drawLine(x1, y1, x2, y2, color);
    drawLine(x2, y2, x0, y0, color);
}

void Display::fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    // Sort vertices by y-coordinate ascending (y0 <= y1 <= y2)
    if (y0 > y1) { swapInt16(y0, y1); swapInt16(x0, x1); }
    if (y1 > y2) { swapInt16(y1, y2); swapInt16(x1, x2); }
    if (y0 > y1) { swapInt16(y0, y1); swapInt16(x0, x1); }

    // Completely skip if triangle is off-screen
    if (y2 < 0 || y0 >= height) return;

    if (y0 == y2) { // All on same line - draw horizontal line
        int16_t a = x0, b = x0;
        if (x1 < a) a = x1;
        else if (x1 > b) b = x1;
        if (x2 < a) a = x2;
        else if (x2 > b) b = x2;

        // Clamp to screen bounds
        if (a < 0) a = 0;
        if (b >= width) b = width - 1;
        if (y0 >= 0 && y0 < height && a <= b) {
            drawLine(a, y0, b, y0, color);
        }
        return;
    }

    // Compute deltas (use int32_t to prevent overflow)
    int32_t dx01 = x1 - x0;
    int32_t dy01 = y1 - y0;
    int32_t dx02 = x2 - x0;
    int32_t dy02 = y2 - y0;
    int32_t dx12 = x2 - x1;
    int32_t dy12 = y2 - y1;

    int32_t sa = 0;
    int32_t sb = 0;

    // Clamp y coordinates to screen
    int16_t yStart = (y0 < 0) ? 0 : y0;
    int16_t yEnd = (y2 >= height) ? height - 1 : y2;

    // For upper part of triangle (from y0 to y1)
    int16_t last = (y1 == y2) ? y1 : y1 - 1;
    if (last >= height) last = height - 1;

    for (int16_t y = yStart; y <= last && y < height; y++) {
        // Adjust sa and sb for clamped start
        if (y == yStart && yStart > y0) {
            sa = dx02 * (yStart - y0);
            sb = dx01 * (yStart - y0);
        }

        int16_t a = x0;
        int16_t b = x0;

        if (dy02 != 0) a = x0 + (sa / dy02);
        if (dy01 != 0) b = x0 + (sb / dy01);

        sa += dx02;
        sb += dx01;

        if (a > b) swapInt16(a, b);

        // Clamp x coordinates to screen
        if (a < 0) a = 0;
        if (b >= width) b = width - 1;

        if (a <= b) {
            drawLine(a, y, b, y, color);
        }
    }

    // For lower part of triangle (from y1 to y2)
    sa = dx12 * (last - y0 + 1);
    sb = dx02 * (last - y0 + 1);

    int16_t yStart2 = (last + 1 < 0) ? 0 : last + 1;
    if (yStart2 <= last) yStart2 = last + 1;

    for (int16_t y = yStart2; y <= yEnd && y < height; y++) {
        // Adjust sa and sb for clamped start
        if (y == yStart2 && yStart2 > last + 1) {
            sa = dx12 * (yStart2 - y0);
            sb = dx02 * (yStart2 - y0);
        }

        int16_t a = x1;
        int16_t b = x0;

        if (dy12 != 0) a = x1 + (sa / dy12);
        if (dy02 != 0) b = x0 + (sb / dy02);

        sa += dx12;
        sb += dx02;

        if (a > b) swapInt16(a, b);

        // Clamp x coordinates to screen
        if (a < 0) a = 0;
        if (b >= width) b = width - 1;

        if (a <= b) {
            drawLine(a, y, b, y, color);
        }
    }
}

void Display::drawArrow(ArrowDirection direction, uint16_t color, uint16_t backgroundColor)
{
    // Clear screen first
    fillScreen(backgroundColor);

    // Wait a bit for the screen to update
    vTaskDelay(pdMS_TO_TICKS(10));

    // Center position
    int16_t cx = width / 2;
    int16_t cy = height / 2;
    int16_t size = 80; // Arrow size

    drawArrow(cx, cy, size, direction, color);
}

void Display::drawArrow(int16_t x, int16_t y, int16_t size, ArrowDirection direction, uint16_t color)
{
    // Arrow shaft dimensions
    int16_t shaftWidth = size / 5;
    int16_t shaftLength = size / 2;
    int16_t headSize = size / 2;

    int dir_val = static_cast<int>(direction);

    if (dir_val == 0) { // UP
        fillRect(x - shaftWidth/2, y, shaftWidth, shaftLength, color);
        fillTriangle(x, y - headSize, x - headSize/2, y, x + headSize/2, y, color);
    }
    else if (dir_val == 1) { // DOWN
        fillRect(x - shaftWidth/2, y - shaftLength, shaftWidth, shaftLength, color);
        fillTriangle(x, y + headSize, x - headSize/2, y, x + headSize/2, y, color);
    }
    else if (dir_val == 2) { // LEFT
        fillRect(x, y - shaftWidth/2, shaftLength, shaftWidth, color);
        fillTriangle(x - headSize, y, x, y - headSize/2, x, y + headSize/2, color);
    }
    else if (dir_val == 3) { // RIGHT
        fillRect(x - shaftLength, y - shaftWidth/2, shaftLength, shaftWidth, color);
        fillTriangle(x + headSize, y, x, y - headSize/2, x, y + headSize/2, color);
    }
    else if (dir_val == 4) { // UP_LEFT
        int16_t offset = size / 2;
        for (int i = -2; i <= 2; i++) {
            drawLine(x + i, y, x - offset + i, y - offset, color);
            drawLine(x, y + i, x - offset, y - offset + i, color);
        }
        fillTriangle(x - offset - headSize/3, y - offset - headSize/3, x - offset + headSize/4, y - offset, x - offset, y - offset + headSize/4, color);
    }
    else if (dir_val == 5) { // UP_RIGHT
        int16_t offset = size / 2;
        for (int i = -2; i <= 2; i++) {
            drawLine(x + i, y, x + offset + i, y - offset, color);
            drawLine(x, y + i, x + offset, y - offset + i, color);
        }
        fillTriangle(x + offset + headSize/3, y - offset - headSize/3, x + offset - headSize/4, y - offset, x + offset, y - offset + headSize/4, color);
    }
    else if (dir_val == 6) { // DOWN_LEFT
        int16_t offset = size / 2;
        for (int i = -2; i <= 2; i++) {
            drawLine(x + i, y, x - offset + i, y + offset, color);
            drawLine(x, y + i, x - offset, y + offset + i, color);
        }
        fillTriangle(x - offset - headSize/3, y + offset + headSize/3, x - offset + headSize/4, y + offset, x - offset, y + offset - headSize/4, color);
    }
    else if (dir_val == 7) { // DOWN_RIGHT
        int16_t offset = size / 2;
        for (int i = -2; i <= 2; i++) {
            drawLine(x + i, y, x + offset + i, y + offset, color);
            drawLine(x, y + i, x + offset, y + offset + i, color);
        }
        fillTriangle(x + offset + headSize/3, y + offset + headSize/3, x + offset - headSize/4, y + offset, x + offset, y + offset - headSize/4, color);
    }
}

// Draw an arrow with shaft and triangular head pointing in a given direction
// angle: 0 = up, 90 = right, 180 = down, 270 = left (compass style)
void Display::drawArrowAtAngle(int16_t cx, int16_t cy, int16_t length, int16_t width, int16_t angle, uint16_t color)
{
    // Convert angle to radians (subtract 90 so 0° = up)
    float angleRad = (angle - 90.0f) * M_PI / 180.0f;

    // Use a different approach - draw the arrow as a series of lines
    // This avoids issues with the fillTriangle function

    float cosAngle = cosf(angleRad);
    float sinAngle = sinf(angleRad);

    // Arrow dimensions
    int16_t shaftLength = (length * 3) / 5;  // 60% for shaft
    int16_t shaftWidth = width;
    int16_t headWidth = width * 2;
    int16_t headLength = (length * 2) / 5;   // 40% for head

    // Draw the shaft as a thick line
    // We'll draw multiple parallel lines to create thickness
    for (int offset = -shaftWidth/2; offset <= shaftWidth/2; offset++) {
        // Calculate perpendicular offset
        float perpX = -sinAngle * offset;
        float perpY = cosAngle * offset;

        // Draw line from base to shaft end
        int16_t x0 = cx + (int16_t)perpX;
        int16_t y0 = cy + (int16_t)perpY;
        int16_t x1 = cx + (int16_t)(cosAngle * shaftLength + perpX);
        int16_t y1 = cy + (int16_t)(sinAngle * shaftLength + perpY);

        drawLine(x0, y0, x1, y1, color);
    }

    // Draw the arrowhead as filled triangle using a different method
    // Calculate the three points of the arrowhead
    int16_t tipX = cx + (int16_t)(cosAngle * length);
    int16_t tipY = cy + (int16_t)(sinAngle * length);

    int16_t baseX = cx + (int16_t)(cosAngle * shaftLength);
    int16_t baseY = cy + (int16_t)(sinAngle * shaftLength);

    // Draw filled arrowhead using lines radiating from tip
    for (int w = -headWidth; w <= headWidth; w++) {
        float t = (float)w / headWidth;  // -1 to 1
        float weight = 1.0f - fabsf(t);  // Triangle shape weight

        // Calculate point along the base of the arrowhead
        float perpX = -sinAngle * w;
        float perpY = cosAngle * w;

        int16_t basePointX = baseX + (int16_t)perpX;
        int16_t basePointY = baseY + (int16_t)perpY;

        // Draw line from this base point toward the tip
        // But only go as far as the triangle shape allows
        int16_t endX = basePointX + (int16_t)((tipX - basePointX) * weight);
        int16_t endY = basePointY + (int16_t)((tipY - basePointY) * weight);

        drawLine(basePointX, basePointY, endX, endY, color);
    }
}

// Alternative implementation using polygon filling
void Display::drawArrowAtAngleAlt(int16_t cx, int16_t cy, int16_t length, int16_t width, int16_t angle, uint16_t color)
{
    // Convert angle to radians (subtract 90 so 0° = up)
    float angleRad = (angle - 90.0f) * M_PI / 180.0f;

    float cosAngle = cosf(angleRad);
    float sinAngle = sinf(angleRad);

    // Arrow dimensions
    int16_t shaftLength = (length * 3) / 5;
    int16_t shaftWidth = width;
    int16_t headWidth = (width * 5) / 2;

    // Draw shaft as a rotated rectangle using scanlines
    // For each row, calculate the intersection with the rotated rectangle
    int16_t minY = cy - length - 10;
    int16_t maxY = cy + length + 10;

    for (int16_t y = minY; y <= maxY; y++) {
        int16_t minX = cx - length - 10;
        int16_t maxX = cx + length + 10;

        int16_t lineStart = -1;
        int16_t lineEnd = -1;

        for (int16_t x = minX; x <= maxX; x++) {
            // Transform point to arrow's local coordinate system
            float dx = x - cx;
            float dy = y - cy;

            // Rotate by negative angle to align with arrow
            float localX = dx * cosAngle + dy * sinAngle;
            float localY = -dx * sinAngle + dy * cosAngle;

            bool inShaft = (localY >= 0 && localY <= shaftLength &&
                           localX >= -shaftWidth/2 && localX <= shaftWidth/2);

            bool inHead = false;
            if (localY >= shaftLength && localY <= length) {
                // Check if in triangular head
                float headProgress = (localY - shaftLength) / (float)(length - shaftLength);
                float maxWidth = headWidth / 2 * (1.0f - headProgress);
                inHead = (localX >= -maxWidth && localX <= maxWidth);
            }

            if (inShaft || inHead) {
                if (lineStart == -1) lineStart = x;
                lineEnd = x;
            } else if (lineStart != -1) {
                // Draw the line segment
                drawLine(lineStart, y, lineEnd, y, color);
                lineStart = -1;
            }
        }

        // Draw any remaining line segment
        if (lineStart != -1) {
            drawLine(lineStart, y, lineEnd, y, color);
        }
    }
}

void Display::drawArrowAtAngle(int16_t angle, uint16_t color, uint16_t backgroundColor)
{
    fillScreen(backgroundColor);

    int16_t cx = width / 2;
    int16_t cy = height / 2;
    int16_t arrowLength = 80;
    int16_t arrowWidth = 8;

    drawArrowAtAngle(cx, cy, arrowLength, arrowWidth, angle, color);
}

void Display::updateArrow(int16_t angle, uint16_t color)
{
    int16_t cx = width / 2;
    int16_t cy = height / 2;
    int16_t arrowLength = 80;
    int16_t arrowWidth = 8;

    if (hasLastArrow && abs(angle - lastArrowAngle) < 5) {
        return;
    }

    if (hasLastArrow) {
        drawArrowAtAngle(cx, cy, arrowLength, arrowWidth, lastArrowAngle, Color::BLACK);
    }

    drawArrowAtAngle(cx, cy, arrowLength, arrowWidth, angle, color);

    lastArrowAngle = angle;
    hasLastArrow = true;
}

void Display::clearArrow()
{
    if (hasLastArrow) {
        int16_t cx = width / 2;
        int16_t cy = height / 2;
        int16_t arrowLength = 80;
        int16_t arrowWidth = 8;

        drawArrowAtAngle(cx, cy, arrowLength, arrowWidth, lastArrowAngle, Color::BLACK);
        hasLastArrow = false;
    }
}

void Display::fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    int16_t x = 0;
    int16_t y = r;
    int16_t d = 1 - r;

    while (x <= y) {
        // Draw horizontal lines for each octant
        drawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
        drawLine(x0 - x, y0 - y, x0 + x, y0 - y, color);
        drawLine(x0 - y, y0 + x, x0 + y, y0 + x, color);
        drawLine(x0 - y, y0 - x, x0 + y, y0 - x, color);

        x++;
        if (d < 0) {
            d += 2 * x + 1;
        } else {
            y--;
            d += 2 * (x - y) + 1;
        }
    }
}

void Display::turnOn()
{
    if (panel_handle) {
        esp_lcd_panel_disp_on_off(panel_handle, true);
    }
}

void Display::turnOff()
{
    if (panel_handle) {
        esp_lcd_panel_disp_on_off(panel_handle, false);
    }
}

void Display::setInverted(bool inverted)
{
    if (panel_handle) {
        esp_lcd_panel_invert_color(panel_handle, inverted);
    }
}

void Display::setMirror(bool mirrorX, bool mirrorY)
{
    if (panel_handle) {
        esp_lcd_panel_mirror(panel_handle, mirrorX, mirrorY);
    }
}

void Display::swapInt16(int16_t &a, int16_t &b)
{
    int16_t t = a;
    a = b;
    b = t;
}

