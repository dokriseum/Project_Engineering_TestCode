#include <pigpio.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#define MAX_TIMINGS 85
#define DHT_PIN 4  // GPIO-Pin für den DHT22

int data[5] = {0, 0, 0, 0, 0};

void read_dht22_data() {
    uint8_t laststate = PI_HIGH;
    uint8_t counter = 0;
    uint8_t j = 0, i;

    data[0] = data[1] = data[2] = data[3] = data[4] = 0;

    // Sende ein Signal, um den DHT22 zu starten
    gpioSetMode(DHT_PIN, PI_OUTPUT);
    gpioWrite(DHT_PIN, PI_LOW);
    usleep(18000);  // 18 ms
    gpioWrite(DHT_PIN, PI_HIGH);
    usleep(40);  // 40 µs
    gpioSetMode(DHT_PIN, PI_INPUT);

    // Lese die Daten vom Sensor
    for (i = 0; i < MAX_TIMINGS; i++) {
        counter = 0;
        while (gpioRead(DHT_PIN) == laststate) {
            counter++;
            if (counter == 255) {
                break;
            }
            usleep(1);
        }
        laststate = gpioRead(DHT_PIN);

        if (counter == 255) break;

        // Ignoriere die ersten 3 Übergänge
        if ((i >= 4) && (i % 2 == 0)) {
            data[j / 8] <<= 1;
            if (counter > 16) data[j / 8] |= 1;
            j++;
        }
    }

    // Prüfe die Prüfsumme
    if ((j >= 40) &&
        (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF))) {
        float t = (data[2] & 0x7F) * 256 + data[3];
        t /= 10.0;
        if (data[2] & 0x80) t *= -1;

        float h = data[0] * 256 + data[1];
        h /= 10.0;

        printf("Luftfeuchtigkeit = %.2f %% Temperatur = %.2f °C\n", h, t);
    } else {
        printf("Fehler beim Lesen der Daten\n");
    }
}

int main(void) {
    if (gpioInitialise() < 0) {
        printf("pigpio konnte nicht initialisiert werden!\n");
        return 1;
    }

    while (1) {
        read_dht22_data();
        sleep(2);  // 2 Sekunden warten
    }

    gpioTerminate();
    return 0;
}