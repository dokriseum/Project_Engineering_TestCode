#include "lcd.hpp"

class MainApplication {
private:
    LCD lcd;

public:
    MainApplication(int lcdAddress) : lcd(lcdAddress) {}

    void run() {
        lcd.clear();

        std::string line1 = "This is line 1 scrolling.";
        std::string line2 = "Line 2 is scrolling too.";
        std::string line3 = "Scrolling on line 3 works!";
        std::string line4 = "Finally, line 4 scrolls!";

        int step = 0;

        while (true) { // Endlosschleife für kontinuierliches Scrollen
            lcd.scrollTextStep(line1, 0, step, 20);
            lcd.scrollTextStep(line2, 1, step, 20);
            lcd.scrollTextStep(line3, 2, step, 20);
            lcd.scrollTextStep(line4, 3, step, 20);

            step++; // Nächster Scroll-Schritt
            usleep(300000); // 300 ms Verzögerung
        }
    }
};

int main() {
    MainApplication app(0x27); // LCD-Adresse
    app.run();
    return 0;
}
