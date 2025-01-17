// Arduino-Sketch: Bodenfeuchtigkeit auslesen und Zahlenwert senden

// Pin für den Bodenfeuchtigkeitssensor
const int sensorPin = A0;

void setup() {
  // Serielle Schnittstelle initialisieren
  Serial.begin(9600);
}

void loop() {
  // Bodenfeuchtigkeit auslesen (analogRead liefert Werte von 0 bis 1023)
  int sensorValue = analogRead(sensorPin);

  // Prozentualer Wert der Bodenfeuchtigkeit berechnen
  int moisturePercent = map(sensorValue, 1023, 0, 0, 100); // 1023 = trocken, 0 = nass

  // Zahlenwert über die serielle Schnittstelle senden
  Serial.println(moisturePercent);

  // 1 Sekunde warten, bevor erneut gemessen wird
  delay(1000);
}