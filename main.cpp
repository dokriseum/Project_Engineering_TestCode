#include "lcd.hpp"

class MainApplication {
private:
    LCD lcd;

public:
    MainApplication(int lcdAddress) : lcd(lcdAddress) {}

    void run() {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Grüße von \x00!"); // Benutzerdefiniertes ß
        std::string longText = "Dies ist ein langer Text mit ß, ä, ö und ü!";
        lcd.scrollText(longText, 1, 300, 20); // Scroll in Zeile 1
    }
};

int main() {
    MainApplication app(0x27); // LCD-Adresse
    app.run();
    return 0;
}
