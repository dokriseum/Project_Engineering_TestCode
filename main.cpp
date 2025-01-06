#include "lcd.hpp"

class MainApplication {
private:
    LCD lcd;

public:
    MainApplication(int lcdAddress) : lcd(lcdAddress) {}
void run() {
    lcd.clear();

    // Zeile 0 scrollen
    lcd.scrollText("This is a example text", 0, 900, 20);

    // Zeile 1 bleibt statisch
    lcd.setCursor(1, 0);
    lcd.print("Huminity: ");

    // Zeile 2 bleibt statisch
    lcd.setCursor(2, 0);
    lcd.print("Temperature: ");

    // Zeile 3 scrollen
    lcd.scrollText("Queer Connect 2025-01-15", 3, 900, 20);
}

    // lcd.print("Testing the first line."); // Benutzerdefiniertes ß
    //std::string line1 = "This is a example text.";
	//std::string line2 = "Malzbier";
	//std::string line3 = "Budweiser";
	//std::string line4 = "Queer Connect 2025-01-15";
    //lcd.scrollText(line1, 0, 900, 20); // Scroll in Zeile 1
	//lcd.scrollText(line2, 1, 900, 20);
	//lcd.scrollText(line3, 2, 900, 20);
	//lcd.scrollText(line4, 3, 900, 20);
    
};

int main() {
    MainApplication app(0x27); // LCD-Adresse
    app.run();
    return 0;
}
