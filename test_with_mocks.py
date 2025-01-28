import pytest
from unittest.mock import MagicMock, patch
from greenhouse_controller import GreenhouseController, HUMIDITY_THRESHOLD, SOIL_MOISTURE_THRESHOLD

@pytest.fixture
def mock_controller():
    """
    Erzeugt eine Controller-Instanz, in der alle Hardware-Aufrufe gemockt werden.
    """
    with patch('greenhouse_controller.adafruit_dht.DHT22') as mock_dht, \
         patch('greenhouse_controller.serial.Serial') as mock_serial, \
         patch('greenhouse_controller.GPIO') as mock_gpio, \
         patch('greenhouse_controller.CharLCD') as mock_lcd:
        
        # Mock-Objekte konfigurieren
        dht_instance = MagicMock()
        mock_dht.return_value = dht_instance
        
        serial_instance = MagicMock()
        mock_serial.return_value = serial_instance
        
        gpio_instance = MagicMock()
        mock_gpio.return_value = gpio_instance
        
        lcd_instance = MagicMock()
        mock_lcd.return_value = lcd_instance
        
        controller = GreenhouseController()
        
        yield controller, dht_instance, serial_instance, mock_gpio, lcd_instance

        # Cleanup
        controller.cleanup()

def test_read_dht22_data_high_humidity(mock_controller):
    """
    Prüft, ob bei hoher Luftfeuchtigkeit der Lüfter aktiviert wird.
    """
    controller, dht_instance, serial_instance, mock_gpio, lcd_instance = mock_controller
    
    # Setup der Sensordaten
    dht_instance.temperature = 25.0
    dht_instance.humidity = HUMIDITY_THRESHOLD + 10  # Über Schwelle
    
    controller.read_dht22_data()
    
    # Prüfen, ob Lüfter an (GPIO LOW)
    mock_gpio.output.assert_any_call(27, mock_gpio.LOW)
    mock_gpio.output.assert_any_call(22, mock_gpio.LOW)

def test_read_dht22_data_low_humidity(mock_controller):
    """
    Prüft, ob bei niedriger Luftfeuchtigkeit der Lüfter deaktiviert wird.
    """
    controller, dht_instance, serial_instance, mock_gpio, lcd_instance = mock_controller
    
    # Setup der Sensordaten
    dht_instance.temperature = 22.0
    dht_instance.humidity = HUMIDITY_THRESHOLD - 10  # Unter Schwelle
    
    controller.read_dht22_data()
    
    # Prüfen, ob Lüfter aus (GPIO HIGH)
    mock_gpio.output.assert_any_call(27, mock_gpio.HIGH)
    mock_gpio.output.assert_any_call(22, mock_gpio.HIGH)

def test_read_soil_moisture_low_moisture(mock_controller):
    """
    Prüft, ob bei zu niedriger Bodenfeuchtigkeit die Pumpe aktiviert wird.
    """
    controller, dht_instance, serial_instance, mock_gpio, lcd_instance = mock_controller
    
    # Arduino gibt Feuchtigkeitswert unterhalb der Schwelle aus
    serial_instance.in_waiting = 1
    serial_instance.readline.return_value = b"10\n"  # 10% Feuchtigkeit
    
    controller.read_soil_moisture()
    
    # Prüfen, ob Pumpe an (GPIO LOW)
    mock_gpio.output.assert_any_call(17, mock_gpio.LOW)

def test_read_soil_moisture_high_moisture(mock_controller):
    """
    Prüft, ob bei ausreichender Bodenfeuchtigkeit die Pumpe deaktiviert wird.
    """
    controller, dht_instance, serial_instance, mock_gpio, lcd_instance = mock_controller
    
    # Arduino gibt Feuchtigkeitswert oberhalb der Schwelle aus
    serial_instance.in_waiting = 1
    serial_instance.readline.return_value = f"{SOIL_MOISTURE_THRESHOLD + 5}\n".encode()
    
    controller.read_soil_moisture()
    
    # Prüfen, ob Pumpe aus (GPIO HIGH)
    mock_gpio.output.assert_any_call(17, mock_gpio.HIGH)
