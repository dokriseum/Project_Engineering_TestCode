#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define MAX_TIMINGS 85
#define DHT_PIN 4  // GPIO4 (WiringPi Pin)

int data[5] = {0, 0, 0, 0, 0};

void read_dht22_data() {
    uint8_t laststate = HIGH;
    uint8_t counter = 0;
    uint8_t j = 0, i;

    data[0] = data[1] = data[2] = data[3] = data[4] = 0;

    // Sende ein Signal, um den DHT22 zu starten
    pinMode(DHT_PIN, OUTPUT);
    digitalWrite(DHT_PIN, LOW);
    delay(18);
    digitalWrite(DHT_PIN, HIGH);
    delayMicroseconds(40);
    pinMode(DHT_PIN, INPUT);

    // Lese die Daten vom Sensor
    for (i = 0; i < MAX_TIMINGS; i++) {
        counter = 0;
        while (digitalRead(DHT_PIN) == laststate) {
            counter++;
            delayMicroseconds(1);
            if (counter == 255) {
                break;
            }
        }
        laststate = digitalRead(DHT_PIN);

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
    if (wiringPiSetup() == -1) {
        printf("WiringPi Setup fehlgeschlagen!\n");
        return 1;
    }

    while (1) {
        read_dht22_data();
        delay(2000);  // 2 Sekunden warten
    }

    return 0;
}
