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
        self.dht_device = adafruit_dht.DHT22(board.D22)
        self.arduino_serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.lcd = CharLCD('PCF8574', config.I2C_ADDRESS, rows=config.LCD_ROWS, cols=config.LCD_COLS)
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

class GrowSystem:
    def __init__(self, hardware, config):
        self.hardware = hardware
        self.config = config
        self.last_display_lines = ["" for _ in range(config.LCD_ROWS)]

    def control_relay(self, relay, state):
        GPIO.output(relay, GPIO.LOW if state else GPIO.HIGH)

    def display_on_lcd(self, lines):
        if lines != self.last_display_lines:
            self.hardware.lcd.clear()
            for i, line in enumerate(lines):
                if i < self.config.LCD_ROWS:
                    self.hardware.lcd.write_string(line[:self.config.LCD_COLS])
                    if i < self.config.LCD_ROWS - 1:
                        self.hardware.lcd.crlf()
            self.last_display_lines = lines

    def control_system(self):
        # Sensorwerte auslesen
        temperature, humidity = self.hardware.read_dht_sensor()
        soil_moisture = self.hardware.read_soil_moisture()

        if temperature is None or humidity is None or soil_moisture is None:
            return

        # Steuerlogik für Bodenfeuchtigkeit
        if soil_moisture < self.config.SOIL_MOISTURE_THRESHOLD:
            self.control_relay(self.config.RELAY_WATER_PUMP, True)
        else:
            self.control_relay(self.config.RELAY_WATER_PUMP, False)

        # Steuerlogik für Luftfeuchtigkeit
        if humidity < self.config.HUMIDITY_THRESHOLD:
            self.control_relay(self.config.RELAY_FAN_1, True)
        else:
            self.control_relay(self.config.RELAY_FAN_1, False)

        # Daten auf LCD anzeigen
        lines = [
            f"Temp: {temperature:.1f} C",
            f"Hum: {humidity:.1f} %",
            f"Soil: {soil_moisture}%"
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