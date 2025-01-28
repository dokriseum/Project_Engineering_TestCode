import time
import adafruit_dht
import RPi.GPIO as GPIO
from RPLCD.i2c import CharLCD
from smbus2 import SMBus

# GPIO Pins
DHT_PIN = 22
PUMP_PIN = 17
FAN_PIN = 27

# Schwellenwerte
TEMP_THRESHOLD = 30.0  # in °C
HUMIDITY_THRESHOLD = 80.0  # in %
SOIL_MOISTURE_THRESHOLD = 30  # in %

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUMP_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(FAN_PIN, GPIO.OUT, initial=GPIO.LOW)

# LCD Setup (I2C Address and Dimensions)
lcd = CharLCD('PCF8574', 0x27, cols=20, rows=4)

# Sensor Setup
DHT_SENSOR = Adafruit_DHT.DHT22

# Bodenfeuchtigkeitssensor Setup
def read_soil_moisture():
    with SMBus(1) as bus:
        raw_value = bus.read_byte(0x48)  # Dummy-Adresse ersetzen, falls nötig
        # Skalierung der Rohwerte in % (anpassen, falls nötig)
        return 100 - (raw_value / 255.0 * 100)

def update_lcd(temp, humidity, soil_moisture):
    lcd.clear()
    lcd.write_string(f"Temp: {temp:.1f}C")
    lcd.crlf()
    lcd.write_string(f"Humidity: {humidity:.1f}%")
    lcd.crlf()
    lcd.write_string(f"Soil Moisture:")
    lcd.crlf()
    lcd.write_string(f"{soil_moisture:.1f}%")

def control_devices(temp, humidity, soil_moisture):
    # Lüftersteuerung
    if temp > TEMP_THRESHOLD or humidity > HUMIDITY_THRESHOLD:
        GPIO.output(FAN_PIN, GPIO.HIGH)
    else:
        GPIO.output(FAN_PIN, GPIO.LOW)

    # Wasserpumpensteuerung
    if soil_moisture < SOIL_MOISTURE_THRESHOLD:
        GPIO.output(PUMP_PIN, GPIO.HIGH)
        time.sleep(5)  # Pumpe für 5 Sekunden laufen lassen
        GPIO.output(PUMP_PIN, GPIO.LOW)

try:
    while True:
        # Temperatur- und Feuchtigkeitsmessung
        humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)

        # Bodenfeuchtigkeitsmessung
        soil_moisture = read_soil_moisture()

        # Fehlerbehandlung bei Sensormessungen
        if humidity is None or temperature is None:
            lcd.clear()
            lcd.write_string("DHT Sensor Error")
            time.sleep(2)
            continue

        # Aktualisiere LCD-Anzeige
        update_lcd(temperature, humidity, soil_moisture)

        # Steuerung der Geräte
        control_devices(temperature, humidity, soil_moisture)

        # Wartezeit zwischen den Messungen
        time.sleep(2)

except KeyboardInterrupt:
    print("Programm beendet.")

finally:
    GPIO.cleanup()
    lcd.clear()
