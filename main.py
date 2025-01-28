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

class GreenhouseController:
    def __init__(self,
                 dht_sensor_pin=board.D22,
                 lcd_address=0x27,
                 lcd_rows=4,
                 lcd_cols=20,
                 arduino_port='/dev/ttyACM0',
                 arduino_baudrate=9600):
        """
        Initialisiert die notwendigen Ressourcen (DHT, Arduino-Serial, GPIO, LCD).
        """
        # Sensor initialisieren (DHT22)
        self.dht_device = adafruit_dht.DHT22(dht_sensor_pin)

        # Serielle Verbindung zum Arduino
        self.arduino_serial = serial.Serial(arduino_port, arduino_baudrate, timeout=1)

        # GPIO initialisieren
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RELAY_WATER_PUMP, GPIO.OUT, initial=GPIO.HIGH)  # Relais aus
        GPIO.setup(RELAY_FAN_1, GPIO.OUT, initial=GPIO.HIGH)       # Relais aus
        GPIO.setup(RELAY_FAN_2, GPIO.OUT, initial=GPIO.HIGH)       # Relais aus

        # LCD initialisieren
        self.lcd = CharLCD('PCF8574', lcd_address, rows=lcd_rows, cols=lcd_cols)
        self.lcd.clear()

    def read_dht22_data(self):
        """
        Liest Temperatur und Luftfeuchtigkeit vom DHT22-Sensor aus
        und steuert entsprechend die Lüfter. Aktualisiert das LCD.
        """
        try:
            temperature = self.dht_device.temperature
            humidity = self.dht_device.humidity

            if temperature is not None and humidity is not None:
                print(f"Temperatur: {temperature:.2f}°C")
                print(f"Luftfeuchtigkeit: {humidity:.2f}%")

                # Lüftersteuerung
                if humidity > HUMIDITY_THRESHOLD:
                    print("Luftfeuchtigkeit zu hoch. Lüfter werden aktiviert.")
                    GPIO.output(RELAY_FAN_1, GPIO.LOW)  # Lüfter 1 einschalten
                    GPIO.output(RELAY_FAN_2, GPIO.LOW)  # Lüfter 2 einschalten
                    fan_status = "Lüfter: AN"
                else:
                    GPIO.output(RELAY_FAN_1, GPIO.HIGH)  # Lüfter 1 ausschalten
                    GPIO.output(RELAY_FAN_2, GPIO.HIGH)  # Lüfter 2 ausschalten
                    fan_status = "Lüfter: AUS"

                # LCD aktualisieren
                self.lcd.cursor_pos = (0, 0)
                self.lcd.write_string(f"Temp: {temperature:.1f}C  Hum: {humidity:.1f}%")
                self.lcd.cursor_pos = (1, 0)
                self.lcd.write_string(f"{fan_status}       ")
            else:
                print("Fehler beim Lesen des DHT22-Sensors. Daten sind None.")

        except RuntimeError as error:
            # Fehler während des Lesens abfangen (z.B. Zeitüberschreitung)
            print(f"Leseproblem DHT22: {error.args[0]}")
        except Exception as error:
            self.dht_device.exit()
            raise error

    def read_soil_moisture(self):
        """
        Liest die Bodenfeuchtigkeit vom Arduino aus und steuert die Wasserpumpe.
        Aktualisiert das LCD.
        """
        try:
            if self.arduino_serial.in_waiting > 0:
                data = self.arduino_serial.readline().decode('utf-8').strip()
                moisture = int(data)
                print(f"Bodenfeuchtigkeit: {moisture}%")

                if moisture < SOIL_MOISTURE_THRESHOLD:
                    print("Bodenfeuchtigkeit zu niedrig. Wasserpumpe wird aktiviert.")
                    GPIO.output(RELAY_WATER_PUMP, GPIO.LOW)  # Wasserpumpe einschalten
                    pump_status = "Pumpe: AN"
                else:
                    GPIO.output(RELAY_WATER_PUMP, GPIO.HIGH)  # Wasserpumpe ausschalten
                    pump_status = "Pumpe: AUS"

                # LCD aktualisieren
                self.lcd.cursor_pos = (2, 0)
                self.lcd.write_string(f"Soil: {moisture}%      ")
                self.lcd.cursor_pos = (3, 0)
                self.lcd.write_string(f"{pump_status}       ")

        except ValueError:
            print("Ungültige Daten vom Arduino empfangen.")
        except Exception as error:
            print(f"Fehler beim Lesen der Bodenfeuchtigkeit: {error}")

    def cleanup(self):
        """
        Schließt und bereinigt alle geöffneten Ressourcen.
        """
        try:
            self.dht_device.exit()
        except:
            pass
        try:
            self.arduino_serial.close()
        except:
            pass
        try:
            GPIO.cleanup()
        except:
            pass
        try:
            self.lcd.clear()
        except:
            pass

def main():
    controller = GreenhouseController()
    try:
        while True:
            controller.read_dht22_data()
            controller.read_soil_moisture()
            time.sleep(2)
    except KeyboardInterrupt:
        print("Programm beendet.")
    finally:
        controller.cleanup()

if __name__ == "__main__":
    main()
