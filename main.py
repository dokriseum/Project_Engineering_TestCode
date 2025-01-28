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
    def __init__(self, dht_pin, relay_pump, relay_fan_1, relay_fan_2, lcd_address, arduino_port):
        self.dht_device = adafruit_dht.DHT22(dht_pin)
        self.relay_pump = relay_pump
        self.relay_fan_1 = relay_fan_1
        self.relay_fan_2 = relay_fan_2
        self.arduino_serial = serial.Serial(arduino_port, 9600, timeout=1)
        self.lcd = CharLCD('PCF8574', lcd_address, rows=4, cols=20)
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.relay_pump, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.relay_fan_1, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.relay_fan_2, GPIO.OUT, initial=GPIO.HIGH)

    def read_dht22_data(self):
        try:
            temperature = self.dht_device.temperature
            humidity = self.dht_device.humidity
            return temperature, humidity
        except RuntimeError as error:
            print(f"DHT22 Error: {error}")
            return None, None

    def read_soil_moisture(self):
        try:
            if self.arduino_serial.in_waiting > 0:
                data = self.arduino_serial.readline().decode('utf-8').strip()
                return int(data)
        except ValueError:
            print("Invalid data from Arduino")
        return None

    def control_fans(self, humidity):
        if humidity > HUMIDITY_THRESHOLD:
            GPIO.output(self.relay_fan_1, GPIO.LOW)
            GPIO.output(self.relay_fan_2, GPIO.LOW)
            return "Fans: ON"
        else:
            GPIO.output(self.relay_fan_1, GPIO.HIGH)
            GPIO.output(self.relay_fan_2, GPIO.HIGH)
            return "Fans: OFF"

    def control_pump(self, soil_moisture):
        if soil_moisture < SOIL_MOISTURE_THRESHOLD:
            GPIO.output(self.relay_pump, GPIO.LOW)
            return "Pump: ON"
        else:
            GPIO.output(self.relay_pump, GPIO.HIGH)
            return "Pump: OFF"

    def update_lcd(self, temperature, humidity, soil_moisture, fan_status, pump_status):
        self.lcd.clear()
        self.lcd.write_string(f"Temp: {temperature:.1f}C  Hum: {humidity:.1f}%")
        self.lcd.crlf()
        self.lcd.write_string(f"Soil: {soil_moisture}%")
        self.lcd.crlf()
        self.lcd.write_string(f"{fan_status}")
        self.lcd.crlf()
        self.lcd.write_string(f"{pump_status}")

    def cleanup(self):
        self.dht_device.exit()
        self.arduino_serial.close()
        GPIO.cleanup()
        self.lcd.clear()

if __name__ == "__main__":
    controller = GreenhouseController(
        dht_pin=board.D22,
        relay_pump=RELAY_WATER_PUMP,
        relay_fan_1=RELAY_FAN_1,
        relay_fan_2=RELAY_FAN_2,
        lcd_address=0x27,
        arduino_port='/dev/ttyACM0'
    )

    try:
        while True:
            temperature, humidity = controller.read_dht22_data()
            soil_moisture = controller.read_soil_moisture()

            if temperature is not None and humidity is not None and soil_moisture is not None:
                fan_status = controller.control_fans(humidity)
                pump_status = controller.control_pump(soil_moisture)
                controller.update_lcd(temperature, humidity, soil_moisture, fan_status, pump_status)

            time.sleep(2)
    except KeyboardInterrupt:
        print("Program terminated.")
    finally:
        controller.cleanup()