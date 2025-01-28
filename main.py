import time
import adafruit_dht
import board
import serial
import RPi.GPIO as GPIO
from RPLCD.i2c import CharLCD

# Konfigurationsklasse
class Config:
    RELAY_WATER_PUMP = 17
    RELAY_FAN_1 = 27

    SOIL_MOISTURE_THRESHOLD = 30  # Prozent
    HUMIDITY_THRESHOLD = 70       # Prozent

    I2C_ADDRESS = 0x27  # Adresse des I2C-LCDs
    LCD_ROWS = 4
    LCD_COLS = 20

# Hardware-Abstraktion
class Hardware:
    def __init__(self, config):
        self.config = config
        self._setup_gpio()
        self.dht_device = adafruit_dht.DHT22(board.D18)
        self.arduino_serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.lcd = CharLCD('PCF8574', config.I2C_ADDRESS, rows=config.LCD_ROWS, cols=config.LCD_COLS)

    def _setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.config.RELAY_WATER_PUMP, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.config.RELAY_FAN_1, GPIO.OUT, initial=GPIO.HIGH)

    def cleanup(self):
        GPIO.cleanup()

# Systemsteuerung
class GrowSystem:
    def __init__(self, hardware, config):
        self.hardware = hardware
        self.config = config

    def read_sensors(self):
        try:
            temperature = self.hardware.dht_device.temperature
            humidity = self.hardware.dht_device.humidity
            return temperature, humidity
        except RuntimeError as e:
            print(f"Fehler beim Lesen der Sensoren: {e}")
            return None, None

    def control_relay(self, relay, state):
        GPIO.output(relay, GPIO.LOW if state else GPIO.HIGH)

    def display_on_lcd(self, lines):
        self.hardware.lcd.clear()
        for i, line in enumerate(lines):
            if i < self.config.LCD_ROWS:
                self.hardware.lcd.write_string(line[:self.config.LCD_COLS])
                if i < self.config.LCD_ROWS - 1:
                    self.hardware.lcd.crlf()

    def control_system(self):
        temperature, humidity = self.read_sensors()
        if temperature is None or humidity is None:
            return

        # Steuerlogik
        if humidity < self.config.HUMIDITY_THRESHOLD:
            self.control_relay(self.config.RELAY_FAN_1, True)
        else:
            self.control_relay(self.config.RELAY_FAN_1, False)

        # Daten auf LCD anzeigen
        lines = [
            f"Temp: {temperature:.1f} C",
            f"Hum: {humidity:.1f} %"
        ]
        self.display_on_lcd(lines)

# Hauptprogramm
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