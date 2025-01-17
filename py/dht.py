#!/usr/bin/env python3

import time
import adafruit_dht
import board

# Sensor initialisieren (DHT22 am GPIO4)
dht_device = adafruit_dht.DHT22(board.D4)

def read_sensor():
	"""Liest Temperatur und Luftfeuchtigkeit vom DHT22-Sensor aus."""
	try:
		# Temperatur und Luftfeuchtigkeit auslesen
		temperature = dht_device.temperature
		humidity = dht_device.humidity
		
		if temperature is not None and humidity is not None:
			print(f"Temperatur: {temperature:.2f}°C")
			print(f"Luftfeuchtigkeit: {humidity:.2f}%")
		else:
			print("Fehler beim Lesen des Sensors. Daten sind None.")
	except RuntimeError as error:
		# Fehler während des Lesens abfangen (z. B. Zeitüberschreitung)
		print(f"Leseproblem: {error.args[0]}")
	except Exception as error:
		dht_device.exit()
		raise error
		
if __name__ == "__main__":
	try:
		while True:
			read_sensor()
			time.sleep(2)  # Wartezeit von 2 Sekunden zwischen den Messungen
	except KeyboardInterrupt:
		print("Programm beendet.")
	finally:
		dht_device.exit()
		