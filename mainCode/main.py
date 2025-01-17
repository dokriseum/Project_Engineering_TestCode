import time
import adafruit_dht
import board
import serial
import RPi.GPIO as GPIO
from RPLCD.i2c import CharLCD

# GPIO-Pins für Relais
RELAY_WATER_PUMP = 17  # GPIO-Pin für Wasserpumpe
RELAY_FAN_1 = 27       # GPIO-Pin für Lüfter 1
RELAY_FAN_2 = 22       # GPIO-Pin für Lüfter 2

# Schwellenwerte
SOIL_MOISTURE_THRESHOLD = 30  # Bodenfeuchtigkeit in Prozent
HUMIDITY_THRESHOLD = 70       # Luftfeuchtigkeit in Prozent

# Sensor initialisieren (DHT22 am GPIO4)
dht_device = adafruit_dht.DHT22(board.D4)

# Serielle Verbindung zum Arduino herstellen
arduino_serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# GPIO initialisieren
GPIO.setmode(GPIO.BCM)
GPIO.setup(RELAY_WATER_PUMP, GPIO.OUT, initial=GPIO.HIGH)  # Relais aus
GPIO.setup(RELAY_FAN_1, GPIO.OUT, initial=GPIO.HIGH)       # Relais aus
GPIO.setup(RELAY_FAN_2, GPIO.OUT, initial=GPIO.HIGH)       # Relais aus

# LCD-Parameter
I2C_ADDRESS = 0x27  # Adresse des I2C-LCDs
LCD_ROWS = 4
LCD_COLS = 20

# LCD initialisieren
lcd = CharLCD('PCF8574', I2C_ADDRESS, rows=LCD_ROWS, cols=LCD_COLS)

try:
    lcd.clear()

    def read_dht22_data():
        """Liest Temperatur und Luftfeuchtigkeit vom DHT22-Sensor aus."""
        try:
            # Temperatur und Luftfeuchtigkeit auslesen
            temperature = dht_device.temperature
            humidity = dht_device.humidity

            if temperature is not None and humidity is not None:
                print(f"Temperatur: {temperature:.2f}°C")
                print(f"Luftfeuchtigkeit: {humidity:.2f}%")

                # Lüftersteuerung basierend auf Luftfeuchtigkeit
                if humidity > HUMIDITY_THRESHOLD:
                    print("Luftfeuchtigkeit zu hoch. Lüfter werden aktiviert.")
                    GPIO.output(RELAY_FAN_1, GPIO.LOW)  # Lüfter 1 einschalten
                    GPIO.output(RELAY_FAN_2, GPIO.LOW)  # Lüfter 2 einschalten
                    fan_status = "Lüfter: AN"
                else:
                    GPIO.output(RELAY_FAN_1, GPIO.HIGH) # Lüfter 1 ausschalten
                    GPIO.output(RELAY_FAN_2, GPIO.HIGH) # Lüfter 2 ausschalten
                    fan_status = "Lüfter: AUS"

                # LCD aktualisieren
                lcd.cursor_pos = (0, 0)
                lcd.write_string(f"Temp: {temperature:.1f}C  Hum: {humidity:.1f}%")
                lcd.cursor_pos = (1, 0)
                lcd.write_string(f"{fan_status}       ")  # Füllt Zeile mit Leerzeichen

            else:
                print("Fehler beim Lesen des DHT22-Sensors. Daten sind None.")
        except RuntimeError as error:
            # Fehler während des Lesens abfangen (z. B. Zeitüberschreitung)
            print(f"Leseproblem DHT22: {error.args[0]}")
        except Exception as error:
            dht_device.exit()
            raise error

    def read_soil_moisture():
        """Liest die Bodenfeuchtigkeit vom Arduino aus."""
        try:
            if arduino_serial.in_waiting > 0:
                data = arduino_serial.readline().decode('utf-8').strip()
                moisture = int(data)
                print(f"Bodenfeuchtigkeit: {moisture}%")

                # Wasserpumpensteuerung basierend auf Bodenfeuchtigkeit
                if moisture < SOIL_MOISTURE_THRESHOLD:
                    print("Bodenfeuchtigkeit zu niedrig. Wasserpumpe wird aktiviert.")
                    GPIO.output(RELAY_WATER_PUMP, GPIO.LOW)  # Wasserpumpe einschalten
                    pump_status = "Pumpe: AN"
                else:
                    GPIO.output(RELAY_WATER_PUMP, GPIO.HIGH) # Wasserpumpe ausschalten
                    pump_status = "Pumpe: AUS"

                # LCD aktualisieren
                lcd.cursor_pos = (2, 0)
                lcd.write_string(f"Soil: {moisture}%      ")
                lcd.cursor_pos = (3, 0)
                lcd.write_string(f"{pump_status}       ")

        except ValueError:
            print("Ungültige Daten vom Arduino empfangen.")
        except Exception as error:
            print(f"Fehler beim Lesen der Bodenfeuchtigkeit: {error}")

    while True:
        read_dht22_data()
        read_soil_moisture()
        time.sleep(2)  # Wartezeit von 2 Sekunden zwischen den Messungen

except KeyboardInterrupt:
    print("Programm beendet.")

finally:
    dht_device.exit()
    arduino_serial.close()
    GPIO.cleanup()
    lcd.clear()
