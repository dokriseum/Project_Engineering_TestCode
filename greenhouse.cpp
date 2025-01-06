#include <wiringPi.h>
#include <iostream>
#include <string>
#include <fcntl.h>      // File control definitions (open, O_RDWR, etc.)
#include <unistd.h>     // UNIX standard function definitions (read, write, close)
#include <termios.h>    // POSIX terminal control definitions
#include <cstring>      // Für memset, etc.
#include <algorithm>    // Für std::remove
#include <DHT.h>        // DHT-Bibliothek für Raspberry Pi (z. B. wiringPi-DHT)

// Pin-Definitionen
#define LED_PIN 0           // GPIO 17 = WiringPi 0
#define DHT_PIN 7           // GPIO 4 = WiringPi 7 (angepasst für DHT)

// Schwellwerte
#define VALUE_REFRESH 256   // Zeit in Millisekunden
#define MIN_SOIL_VALUE 0    // Minimaler Bodenfeuchtigkeitswert
#define MAX_SOIL_VALUE 1023 // Maximaler Bodenfeuchtigkeitswert

// Globale Variablen
int zahler = 1;
const char* serialPort = "/dev/ttyACM0"; // Pfad zur seriellen Schnittstelle
int fd; // Datei-Deskriptor für die serielle Verbindung
std::string serialBuffer; // Puffer für serielle Daten

dht DHT; // Instanz für den DHT-Sensor

// Funktionen
void checkZahler() {
    zahler++;
    if (zahler > 127) { // INT8_MAX
        zahler = 1;
    }
}

void printSerial(int soilValue, float humidity, float temperature) {
    std::cout << "Counter: " << zahler << "/127  |  "
              << "Soil Moisture Value: " << soilValue << "  |  "
              << "Air Humidity: " << humidity << "%  |  "
              << "Temperature: " << temperature << "°C" << std::endl;
}

void readTempAirHumidity(float &humidity, float &temperature) {
    int result = DHT.read22(DHT_PIN); // Lese Daten vom DHT22
    if (result == DHTLIB_OK) {
        humidity = DHT.humidity;
        temperature = DHT.temperature;
    } else {
        std::cerr << "Fehler beim Lesen des DHT-Sensors: ";
        switch (result) {
            case DHTLIB_ERROR_CHECKSUM:
                std::cerr << "Checksum-Fehler";
                break;
            case DHTLIB_ERROR_TIMEOUT:
                std::cerr << "Timeout-Fehler";
                break;
            default:
                std::cerr << "Unbekannter Fehler";
        }
        std::cerr << std::endl;

        humidity = -1.0; // Fehlerwert
        temperature = -1.0; // Fehlerwert
    }
}

int readSoilValueFromSerial() {
    char buffer[256];
    memset(buffer, 0, sizeof(buffer));

    int n = read(fd, &buffer, sizeof(buffer) - 1);
    if (n > 0) {
        serialBuffer.append(buffer, n);

        // Verarbeite vollständige Zeilen
        size_t pos;
        while ((pos = serialBuffer.find('\n')) != std::string::npos) {
            std::string line = serialBuffer.substr(0, pos);
            serialBuffer.erase(0, pos + 1);

            // Entferne Carriage Return, falls vorhanden
            line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());

            try {
                int value = std::stoi(line);

                // Validierungscheck für den Sensorwert
                if (value >= MIN_SOIL_VALUE && value <= MAX_SOIL_VALUE) {
                    return value;
                } else {
                    std::cerr << "Ungültiger Wert außerhalb des Bereichs: " << value << std::endl;
                }
            } catch (const std::exception &e) {
                std::cerr << "Fehler beim Konvertieren des Werts: " << e.what() << " (Empfangen: " << line << ")" << std::endl;
            }
        }
    }

    return -1; // Fehlerwert
}

void setupSerialConnection() {
    fd = open(serialPort, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Kann " << serialPort << " nicht öffnen!\n";
        exit(1);
    }

    termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Fehler bei tcgetattr.\n";
        close(fd);
        exit(1);
    }

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag &= ~PARENB;          // Parität aus
    tty.c_cflag &= ~CSTOPB;          // 1 Stoppbit
    tty.c_cflag &= ~CSIZE;           // Bits pro Byte ...
    tty.c_cflag |=  CS8;             // ... auf 8 Bits setzen
    tty.c_cflag &= ~CRTSCTS;         // Keine Hardware-Flow-Control
    tty.c_cflag |=  CREAD | CLOCAL;  // Receiver einschalten, Local Mode

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // keine Software-Flow-Control

    tty.c_oflag &= ~OPOST; // raw output

    tcsetattr(fd, TCSANOW, &tty);
}

int main() {
    // GPIO-Setup
    wiringPiSetup();
    pinMode(LED_PIN, OUTPUT);

    // Serielle Verbindung einrichten
    setupSerialConnection();

    // Hauptschleife
    try {
        while (true) {
            // Bodenfeuchtigkeit über serielle Verbindung lesen
            int soilValue = readSoilValueFromSerial();
            if (soilValue == -1) {
                continue;
            }

            // LED-Steuerung
            if (soilValue > 500) { // Beispiel-Schwellwert für trocken
                digitalWrite(LED_PIN, HIGH);
            } else { // Feucht
                digitalWrite(LED_PIN, LOW);
            }

            // Temperatur- und Luftfeuchtigkeitsdaten lesen
            float humidity = 0.0, temperature = 0.0;
            readTempAirHumidity(humidity, temperature);

            // Ausgabe
            printSerial(soilValue, humidity, temperature);

            // Zähler prüfen
            checkZahler();

            // Pause
            usleep(VALUE_REFRESH * 1000); // in Millisekunden
        }
    } catch (...) {
        std::cerr << "Programm beendet." << std::endl;
    }

    // Serielle Verbindung schließen
    close(fd);

    return 0;
}
