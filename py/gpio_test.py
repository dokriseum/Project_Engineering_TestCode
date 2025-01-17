import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)  # Beispiel: Pin für Wasserpumpe
GPIO.output(17, GPIO.HIGH)
print("GPIO Test erfolgreich!")
GPIO.cleanup()
