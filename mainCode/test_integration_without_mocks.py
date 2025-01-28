import pytest
import time
import sys
import os

# Verzeichnis des aktuellen Skripts
current_dir = os.path.dirname(os.path.abspath(__file__))
# Verzeichnis übergeordnete Ebene
parent_dir = os.path.dirname(current_dir)
# Dem Python-Pfad hinzufügen
sys.path.append(parent_dir)

from greenhouse_controller import GreenhouseController

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
    # Hier keine konkrete Prüfung, da Real-Hardware. 
    # Man könnte z.B. nur prüfen, ob kein Exception geworfen wurde.
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
