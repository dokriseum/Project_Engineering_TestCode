#include "lcd.hpp"

LCD::LCD(int address, int backlightOn) : addr(address), backlight(backlightOn) {
    fd = wiringPiI2CSetup(addr);
    if (fd == -1) {
        std::cerr << "Error initializing I2C device at address 0x" << std::hex << addr << std::endl;
        exit(1);
    }
    initialize();
    loadCustomChars(); // Lade benutzerdefinierte Zeichen wie ß
}

void LCD::writeByte(uint8_t data) {
    wiringPiI2CWrite(fd, data | backlight);
    usleep(50);
}

void LCD::toggleEnable(uint8_t data) {
    usleep(500);
    writeByte(data | 0x04);
    usleep(500);
    writeByte(data & ~0x04);
    usleep(500);
}

void LCD::sendCommand(uint8_t cmd) {
    writeByte(cmd & 0xF0);
    toggleEnable(cmd & 0xF0);
    writeByte((cmd << 4) & 0xF0);
    toggleEnable((cmd << 4) & 0xF0);
}

void LCD::sendData(uint8_t data) {
    writeByte((data & 0xF0) | 0x01);
    toggleEnable((data & 0xF0) | 0x01);
    writeByte(((data << 4) & 0xF0) | 0x01);
    toggleEnable(((data << 4) & 0xF0) | 0x01);
}

void LCD::initialize() {
    sendCommand(0x33);
    sendCommand(0x32);
    sendCommand(0x28);
    sendCommand(0x0C);
    sendCommand(0x06);
    sendCommand(0x01);
    usleep(2000);
}

void LCD::clear() {
    sendCommand(0x01);
    usleep(2000);
}

void LCD::setCursor(int row, int col) {
    int rowOffsets[] = { 0x00, 0x40, 0x14, 0x54 };
    sendCommand(0x80 | (col + rowOffsets[row]));
}

void LCD::print(const std::string &text) {
    for (char c : text) {
        sendData(c);
    }
}

void LCD::scrollText(const std::string &text, int row, int delayMs, int lcdWidth) {
    std::string paddedText = text + " ";
    for (size_t i = 0; i < paddedText.size(); i++) {
        std::string window = paddedText.substr(i, lcdWidth);
        if (window.size() < lcdWidth) {
            window += paddedText.substr(0, lcdWidth - window.size());
        }
        setCursor(row, 0);
        print(window);
        usleep(delayMs * 1000);
        if (i == paddedText.size() - 1) {
            i = -1; // Zurück zum Anfang
        }
    }
}

void LCD::createCustomChar(int location, uint8_t charmap[]) {
    location &= 0x7;
    sendCommand(0x40 | (location << 3));
    for (int i = 0; i < 8; i++) {
        sendData(charmap[i]);
    }
}

void LCD::loadCustomChars() {
    uint8_t ssChar[8] = {
        0b00000,
        0b01010,
        0b01010,
        0b01110,
        0b01010,
        0b01010,
        0b01110,
        0b00000
    };
    createCustomChar(0, ssChar); // Speichert `ß` in CGRAM-Slot 0
}
