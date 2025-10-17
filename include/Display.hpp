#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"

// Arrow directions (for 8-way directional arrows)
enum class ArrowDirection {
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3,
    UP_LEFT = 4,
    UP_RIGHT = 5,
    DOWN_LEFT = 6,
    DOWN_RIGHT = 7
};

// Common RGB565 colors
namespace Color {
    constexpr uint16_t RED     = 0xF800;
    constexpr uint16_t GREEN   = 0x07E0;
    constexpr uint16_t BLUE    = 0x001F;
    constexpr uint16_t WHITE   = 0xFFFF;
    constexpr uint16_t BLACK   = 0x0000;
    constexpr uint16_t YELLOW  = 0xFFE0;
    constexpr uint16_t CYAN    = 0x07FF;
    constexpr uint16_t MAGENTA = 0xF81F;
    constexpr uint16_t ORANGE  = 0xFC00;
    constexpr uint16_t GRAY    = 0x8410;
}

class Display {
public:
    Display();
    ~Display();

    // Initialize the display
    bool init();

    // Basic drawing functions
    void fillScreen(uint16_t color);
    void drawPixel(int16_t x, int16_t y, uint16_t color);
    void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
    void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
    void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
    void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
    void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);

    // 8-way directional arrow (UP, DOWN, LEFT, RIGHT, diagonals)
    void drawArrow(ArrowDirection direction, uint16_t color, uint16_t backgroundColor = Color::BLACK);
    void drawArrow(int16_t x, int16_t y, int16_t size, ArrowDirection direction, uint16_t color);

    // Draw arrow at any angle (0-360 degrees, 0=up, clockwise)
    void drawArrowAtAngle(int16_t angle, uint16_t color, uint16_t backgroundColor = Color::BLACK);
    void drawArrowAtAngle(int16_t cx, int16_t cy, int16_t arrowLength, int16_t baseWidth, int16_t angle, uint16_t color);
    void drawArrowAtAngleAlt(int16_t cx, int16_t cy, int16_t length, int16_t width, int16_t angle, uint16_t color);


    // Erase previous arrow and draw new one (for smooth animation)
    void updateArrow(int16_t angle, uint16_t color);
    void clearArrow();

    // Display control
    void setBrightness(uint8_t brightness); // 0-100
    void setInverted(bool inverted);
    void setMirror(bool mirrorX, bool mirrorY);
    void turnOn();
    void turnOff();

    // Get display dimensions
    int16_t getWidth() const { return width; }
    int16_t getHeight() const { return height; }

private:
    esp_lcd_panel_handle_t panel_handle;
    esp_lcd_panel_io_handle_t io_handle;
    spi_device_handle_t spi_handle;

    uint16_t *framebuffer;
    int16_t width;
    int16_t height;

    // Track last arrow angle for erasing
    int16_t lastArrowAngle;
    bool hasLastArrow;

    // Helper functions
    void drawBitmap(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *data);
    void swapInt16(int16_t &a, int16_t &b);
};

#endif // DISPLAY_HPP

