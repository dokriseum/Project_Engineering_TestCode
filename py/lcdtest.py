from RPLCD.i2c import CharLCD
from time import sleep

# LCD-Parameter
I2C_ADDRESS = 0x27  # Ersetze 0x27 durch die Adresse deines LCDs (z.B. aus i2cdetect -y 1)
LCD_ROWS = 4        # Anzahl der Reihen
LCD_COLS = 20       # Anzahl der Spalten

# LCD initialisieren
lcd = CharLCD('PCF8574', I2C_ADDRESS, rows=LCD_ROWS, cols=LCD_COLS)

try:
    # LCD löschen
    lcd.clear()

    # Begrüßungstext anzeigen
    lcd.write_string("LCD-Test\n")
    lcd.write_string("Zeile 2\n")
    lcd.write_string("Zeile 3\n")
    lcd.write_string("Zeile 4")
    
    sleep(5)  # Warte 5 Sekunden

    # Weitere Tests: Dynamisches Schreiben
    for i in range(1, 5):
        lcd.clear()
        lcd.write_string(f"Zeile {i}: Test")
        sleep(2)

    lcd.clear()
    lcd.write_string("Test abgeschlossen")
    sleep(3)

finally:
    # LCD zurücksetzen
    lcd.clear()
