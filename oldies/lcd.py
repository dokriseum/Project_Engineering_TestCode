import smbus2
import time

class LCD:
    def __init__(self, i2c_addr, i2c_bus=1):
        self.addr = i2c_addr
        self.bus = smbus2.SMBus(i2c_bus)
        self.backlight = 0x08  # Hintergrundbeleuchtung aktivieren
        self.enable = 0x04     # Enable-Bit

        self.init_lcd()

    def write_byte(self, data):
        self.bus.write_byte(self.addr, data | self.backlight)

    def toggle_enable(self, data):
        time.sleep(0.0005)
        self.write_byte(data | self.enable)
        time.sleep(0.0005)
        self.write_byte(data & ~self.enable)
        time.sleep(0.0005)

    def send_command(self, cmd):
        self.write_byte(cmd & 0xF0)  # High nibble
        self.toggle_enable(cmd & 0xF0)
        self.write_byte((cmd << 4) & 0xF0)  # Low nibble
        self.toggle_enable((cmd << 4) & 0xF0)

    def send_data(self, data):
        self.write_byte((data & 0xF0) | 0x01)  # High nibble, RS=1
        self.toggle_enable((data & 0xF0) | 0x01)
        self.write_byte(((data << 4) & 0xF0) | 0x01)  # Low nibble, RS=1
        self.toggle_enable(((data << 4) & 0xF0) | 0x01)

    def init_lcd(self):
        self.send_command(0x33)  # Initialize
        self.send_command(0x32)  # Set to 4-bit mode
        self.send_command(0x06)  # Cursor move direction
        self.send_command(0x0C)  # Turn cursor off
        self.send_command(0x28)  # 2 line display
        self.send_command(0x01)  # Clear display
        time.sleep(0.002)        # Wait for clear

    def clear(self):
        self.send_command(0x01)
        time.sleep(0.002)

    def set_cursor(self, row, col):
        row_offsets = [0x00, 0x40, 0x14, 0x54]
        self.send_command(0x80 | (col + row_offsets[row]))

    def print(self, text):
        for char in text:
            self.send_data(ord(char))


# Hauptprogramm
if __name__ == "__main__":
    lcd_address = 0x27  # I2C-Adresse des Displays
    lcd = LCD(lcd_address)

    lcd.clear()
    lcd.set_cursor(0, 0)
    lcd.print("Line 1: Hello, LCD!")

    lcd.set_cursor(1, 0)
    lcd.print("Line 2: How are you?")

    lcd.set_cursor(2, 0)
    lcd.print("Line 3: All is OK.")

    lcd.set_cursor(3, 0)
    lcd.print("Line 4: Have fun!")
