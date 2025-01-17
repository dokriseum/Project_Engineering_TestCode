#!/usr/bin/env python3

import time
import adafruit_dht
import board
import serial

# Sensor initialisieren (DHT22 am GPIO4)
dht_device = adafruit_dht.DHT22(board.D4)

# Serielle Verbindung zum Arduino herstellen
arduino_serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

def read_dht22_data():
	"""Liest Temperatur und Luftfeuchtigkeit vom DHT22-Sensor aus."""
	try:
		# Temperatur und Luftfeuchtigkeit auslesen
		temperature = dht_device.temperature
		humidity = dht_device.humidity
		
		if temperature is not None and humidity is not None:
			print(f"Temperatur: {temperature:.2f}°C")
			print(f"Luftfeuchtigkeit: {humidity:.2f}%")
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
			print(f"Bodenfeuchtigkeit: {data}%")
	except Exception as error:
		print(f"Fehler beim Lesen der Bodenfeuchtigkeit: {error}")
		
if __name__ == "__main__":
	try:
		while True:
			read_dht22_data()
			read_soil_moisture()
			time.sleep(2)  # Wartezeit von 2 Sekunden zwischen den Messungen
	except KeyboardInterrupt:
		print("Programm beendet.")
	finally:
		dht_device.exit()
		arduino_serial.close()
		