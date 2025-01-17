from RPLCD.i2c import CharLCD
from time import sleep

# LCD-Parameter
I2C_ADDRESS = 0x27  # Ersetze mit deiner I2C-Adresse (z. B. 0x27 oder 0x3F)
LCD_ROWS = 4        # Anzahl der Reihen
LCD_COLS = 20       # Anzahl der Spalten

# LCD initialisieren
lcd = CharLCD('PCF8574', I2C_ADDRESS, rows=LCD_ROWS, cols=LCD_COLS)

try:
    # LCD löschen
    lcd.clear()

    # Test: Schreibe Text in alle Zeilen
    lcd.cursor_pos = (0, 0)  # Erste Zeile
    lcd.write_string("Zeile 1: Hallo!")

    lcd.cursor_pos = (1, 0)  # Zweite Zeile
    lcd.write_string("Zeile 2: Test")

    lcd.cursor_pos = (2, 0)  # Dritte Zeile
    lcd.write_string("Zeile 3: LCD")

    lcd.cursor_pos = (3, 0)  # Vierte Zeile
    lcd.write_string("Zeile 4: Erfolg")

    # Warte 5 Sekunden, um die Ausgabe zu sehen
    sleep(5)

finally:
    # LCD zurücksetzen
    lcd.clear()
