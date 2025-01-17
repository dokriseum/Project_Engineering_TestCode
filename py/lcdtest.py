from RPLCD.i2c import CharLCD
from time import sleep

# LCD-Parameter
I2C_ADDRESS = 0x27  # Ersetze 0x27 mit der Adresse deines LCDs
LCD_ROWS = 4        # Anzahl der Reihen
LCD_COLS = 20       # Anzahl der Spalten

# LCD initialisieren
lcd = CharLCD('PCF8574', I2C_ADDRESS, rows=LCD_ROWS, cols=LCD_COLS)

try:
    # LCD löschen
    lcd.clear()

    # Text gezielt in einzelne Zeilen schreiben
    lcd.cursor_pos = (0, 0)  # Erste Zeile, erste Spalte
    lcd.write_string("Zeile 1: Hallo Welt")

    lcd.cursor_pos = (1, 0)  # Zweite Zeile, erste Spalte
    lcd.write_string("Zeile 2: Test")

    lcd.cursor_pos = (2, 5)  # Dritte Zeile, sechste Spalte
    lcd.write_string("Zeile 3: Mitte")

    lcd.cursor_pos = (3, 10)  # Vierte Zeile, elfte Spalte
    lcd.write_string("Zeile 4: Rechts")

    sleep(5)  # Warte 5 Sekunden, damit der Text sichtbar bleibt

finally:
    # LCD zurücksetzen
    lcd.clear()
