#include <wiringPiI2C.h>
#include <unistd.h>
#include <iostream>
#include <string>

class LCD {
private:
    int fd;
    int addr;
    int backlight;

    void writeByte(uint8_t data) {
        wiringPiI2CWrite(fd, data | backlight);
        usleep(50);
    }

    void toggleEnable(uint8_t data) {
        usleep(500);
        writeByte(data | 0x04); // Enable = 1
        usleep(500);
        writeByte(data & ~0x04); // Enable = 0
        usleep(500);
    }

    void sendCommand(uint8_t cmd) {
        writeByte(cmd & 0xF0);
        toggleEnable(cmd & 0xF0);
        writeByte((cmd << 4) & 0xF0);
        toggleEnable((cmd << 4) & 0xF0);
    }

    void sendData(uint8_t data) {
        writeByte((data & 0xF0) | 0x01);
        toggleEnable((data & 0xF0) | 0x01);
        writeByte(((data << 4) & 0xF0) | 0x01);
        toggleEnable(((data << 4) & 0xF0) | 0x01);
    }

public:
    LCD(int address, int backlightOn = 0x08) : addr(address), backlight(backlightOn) {
        fd = wiringPiI2CSetup(addr);
        if (fd == -1) {
            std::cerr << "Error initializing I2C device at address 0x" << std::hex << addr << std::endl;
            exit(1);
        }
        initialize();
    }

    void initialize() {
        sendCommand(0x33);
        sendCommand(0x32);
        sendCommand(0x28);
        sendCommand(0x0C);
        sendCommand(0x06);
        sendCommand(0x01);
        usleep(2000);
    }

    void clear() {
        sendCommand(0x01);
        usleep(2000);
    }

    void setCursor(int row, int col) {
        int rowOffsets[] = { 0x00, 0x40, 0x14, 0x54 };
        sendCommand(0x80 | (col + rowOffsets[row]));
    }

    void print(const std::string &text) {
        std::string mappedText = mapSpecialChars(text);
        for (char c : mappedText) {
            sendData(c);
        }
    }

    std::string mapSpecialChars(const std::string &text) {
        std::string mappedText;
        for (size_t i = 0; i < text.size(); ++i) {
            unsigned char c = text[i];
            if (c == 0xC3 && i + 1 < text.size()) {
                // Sonderzeichen erkennen
                unsigned char next = text[i + 1];
                switch (next) {
                    case 0xA4: mappedText += '\xE1'; i++; break; // ä
                    case 0xB6: mappedText += '\xEF'; i++; break; // ö
                    case 0xBC: mappedText += '\xF5'; i++; break; // ü
                    case 0x9F: mappedText += '\x00'; i++; break; // ß (benutzerdefiniertes Zeichen)
                    default: mappedText += c; break;
                }
            } else {
                // Normale Zeichen hinzufügen
                mappedText += c;
            }
        }
        return mappedText;
    }

    void loadCustomChars() {
        uint8_t ssChar[8] = {
            0b00000,
            0b01010,
            0b01010,
            0b01010,
            0b01100,
            0b01010,
            0b01010,
            0b00000
        };
        createCustomChar(0, ssChar); // Speichere 'ß' in CGRAM
    }

    void createCustomChar(int location, uint8_t charmap[]) {
        location &= 0x7; // Nur 8 CGRAM-Plätze verfügbar
        sendCommand(0x40 | (location << 3)); // Setze CGRAM-Adresse
        for (int i = 0; i < 8; i++) {
            sendData(charmap[i]);
        }
    }
};

int main() {
    int lcdAddress = 0x27; // LCD-Adresse
    LCD lcd(lcdAddress);

    lcd.initialize();
    lcd.loadCustomChars();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Grüße von ä, ö, ü");
    lcd.setCursor(1, 0);
    lcd.print("Und ß ist dabei!");

    return 0;
}
