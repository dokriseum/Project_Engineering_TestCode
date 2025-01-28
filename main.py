import time
import adafruit_dht
import board
import serial
import RPi.GPIO as GPIO
from RPLCD.i2c import CharLCD

class Config:
    # GPIO-Pins für Relais
    RELAY_WATER_PUMP = 17
    RELAY_FAN_1 = 27

    # Schwellenwerte
    SOIL_MOISTURE_THRESHOLD = 30  # Bodenfeuchtigkeit in Prozent
    HUMIDITY_THRESHOLD = 70       # Luftfeuchtigkeit in Prozent

    # LCD-Parameter
    I2C_ADDRESS = 0x27  # Adresse des I2C-LCDs
    LCD_ROWS = 4
    LCD_COLS = 20

class Hardware:
    def __init__(self, config):
        self.config = config
        self.dht_device = adafruit_dht.DHT22(board.D18)
        self.arduino_serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        try:
            self.lcd = CharLCD('PCF8574', config.I2C_ADDRESS, rows=config.LCD_ROWS, cols=config.LCD_COLS)
            print("LCD erfolgreich initialisiert.")
            self.test_lcd()
        except Exception as e:
            print(f"Fehler bei der Initialisierung des LCDs: {e}")
            self.lcd = None
        self._setup_gpio()

    def _setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.config.RELAY_WATER_PUMP, GPIO.OUT, initial=GPIO.HIGH)  # Relais aus
        GPIO.setup(self.config.RELAY_FAN_1, GPIO.OUT, initial=GPIO.HIGH)       # Relais aus

    def cleanup(self):
        GPIO.cleanup()

    def read_dht_sensor(self):
        try:
            temperature = self.dht_device.temperature
            humidity = self.dht_device.humidity
            return temperature, humidity
        except RuntimeError:
            return None, None

    def read_soil_moisture(self):
        self.arduino_serial.write(b"read_soil_moisture\n")
        line = self.arduino_serial.readline().decode().strip()
        try:
            return int(line)
        except ValueError:
            return None

    def test_lcd(self):
        """Testet die LCD-Funktion durch Schreiben von Testnachrichten in alle Zeilen."""
        if self.lcd:
            try:
                self.lcd.clear()
                self.lcd.cursor_pos = (0, 0)
                self.lcd.write_string("Zeile 1: Test")
                self.lcd.cursor_pos = (1, 0)
                self.lcd.write_string("Zeile 2: Test")
                self.lcd.cursor_pos = (2, 0)
                self.lcd.write_string("Zeile 3: Test")
                self.lcd.cursor_pos = (3, 0)
                self.lcd.write_string("Zeile 4: Test")
                time.sleep(5)
                self.lcd.clear()
            except Exception as e:
                print(f"Fehler beim Testen des LCDs: {e}")

class GrowSystem:
    def __init__(self, hardware, config):
        self.hardware = hardware
        self.config = config
        self.last_display_lines = ["" for _ in range(config.LCD_ROWS)]

    def control_relay(self, relay, state):
        GPIO.output(relay, GPIO.LOW if state else GPIO.HIGH)

    def sanitize_data(self, data):
        """Säubert Daten, um Fehler auf dem LCD zu vermeiden."""
        if data is None:
            return "---"
        if isinstance(data, (float, int)):
            return f"{data:.1f}" if isinstance(data, float) else str(data)
        return str(data)[:self.config.LCD_COLS]

    def display_on_lcd(self, lines):
        if self.hardware.lcd is None:
            print("LCD ist nicht initialisiert. Keine Ausgabe möglich.")
            return

        if lines != self.last_display_lines:
            try:
                for i, line in enumerate(lines):
                    if i < self.config.LCD_ROWS:
                        self.hardware.lcd.cursor_pos = (i, 0)
                        self.hardware.lcd.write_string(line.ljust(self.config.LCD_COLS))
                self.last_display_lines = lines
                print("Auf LCD ausgegeben:", lines)
            except Exception as e:
                print(f"Fehler bei der Ausgabe auf das LCD: {e}")

    def control_system(self):
        # Sensorwerte auslesen
        temperature, humidity = self.hardware.read_dht_sensor()
        soil_moisture = self.hardware.read_soil_moisture()

        temperature_display = self.sanitize_data(temperature)
        humidity_display = self.sanitize_data(humidity)
        soil_moisture_display = self.sanitize_data(soil_moisture)

        # Steuerlogik für Bodenfeuchtigkeit
        if soil_moisture is not None and soil_moisture < self.config.SOIL_MOISTURE_THRESHOLD:
            self.control_relay(self.config.RELAY_WATER_PUMP, True)
        else:
            self.control_relay(self.config.RELAY_WATER_PUMP, False)

        # Steuerlogik für Luftfeuchtigkeit
        if humidity is not None and humidity < self.config.HUMIDITY_THRESHOLD:
            self.control_relay(self.config.RELAY_FAN_1, True)
        else:
            self.control_relay(self.config.RELAY_FAN_1, False)

        # Daten auf LCD anzeigen
        lines = [
            f"Temp: {temperature_display} C",
            f"Hum: {humidity_display} %",
            f"Soil: {soil_moisture_display}%"
        ]
        self.display_on_lcd(lines)

if __name__ == "__main__":
    config = Config()
    hardware = Hardware(config)
    grow_system = GrowSystem(hardware, config)

    try:
        while True:
            grow_system.control_system()
            time.sleep(2)
    except KeyboardInterrupt:
        print("Beende das System...")
    finally:
        hardware.cleanup()
