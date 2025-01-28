# test_integration_without_mocks.py
import pytest
import time
from main.greenhouse_controller import GreenhouseController

@pytest.fixture(scope="module")
def controller():
    """
    Erstellt eine Instanz des GreenhouseControllers
    und gibt sie nach dem Testen wieder frei.
    """
    ctrl = GreenhouseController()
    yield ctrl
    ctrl.cleanup()

def test_dht22_data(controller):
    """
    Integrationstest für DHT22 (Voraussetzung: echter Sensor angeschlossen).
    """
    controller.read_dht22_data()
    assert True

def test_soil_moisture(controller):
    """
    Integrationstest für Bodenfeuchtigkeit (Voraussetzung: echtes Arduino-Setup).
    """
    # Hier wird davon ausgegangen, dass das Arduino was sendet.
    # Ohne angeschlossenes Arduino schlägt der Test fehl.
    controller.read_soil_moisture()
    assert True

def test_cleanup_no_exceptions(controller):
    """
    Testet, ob das Bereinigen keine Fehler wirft.
    """
    # Nochmals aufrufen, um sicherzustellen, dass kein Fehler bei mehrfachaufruf passiert.
    controller.cleanup()
    assert True
