#ifndef LCD_HPP
#define LCD_HPP

#include <wiringPiI2C.h>
#include <unistd.h>
#include <string>
#include <iostream>

class LCD {
private:
    int fd;
    int addr;
    int backlight;

    void writeByte(uint8_t data);
    void toggleEnable(uint8_t data);
    void sendCommand(uint8_t cmd);
    void sendData(uint8_t data);

    void createCustomChar(int location, uint8_t charmap[]);

public:
    LCD(int address, int backlightOn = 0x08);
    void initialize();
    void clear();
    void setCursor(int row, int col);
    void print(const std::string &text);
    void scrollText(const std::string &text, int row, int delayMs, int lcdWidth);
    void loadCustomChars();
};

#endif
