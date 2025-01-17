import serial
import time

# Serielle Verbindung initialisieren
arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Passe den Port an, falls notwendig
time.sleep(2)  # Warte, bis die Verbindung hergestellt ist

try:
    while True:
        if arduino.in_waiting > 0:  # Prüfen, ob Daten vorhanden sind
            data = arduino.readline().decode('utf-8').strip()
            print(f"Empfangen: {data}")  # Empfangene Daten anzeigen
except KeyboardInterrupt:
    print("Programm beendet.")
finally:
    arduino.close()
