#include <iostream>
#include <string>
#include <fcntl.h>      // File control definitions (open, O_RDWR, etc.)
#include <unistd.h>     // UNIX standard function definitions (read, write, close)
#include <termios.h>    // POSIX terminal control definitions
#include <cstring>      // Für memset, etc.

int main() {
    // Pfad zu deinem seriellen Gerät (bei Arduino Uno/Leonardo in der Regel /dev/ttyACM0)
    const char* serialPort = "/dev/ttyACM0"; 

    // 1) Seriellen Port öffnen
    int fd = open(serialPort, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1) {
        std::cerr << "Kann " << serialPort << " nicht öffnen!\n";
        return 1;
    }

    // 2) Port-Konfiguration (Baudrate, 8N1, etc.)
    termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Fehler bei tcgetattr.\n";
        close(fd);
        return 1;
    }

    // Baudrate auf 9600 setzen (entsprechend deinem Arduino-Serial.begin(9600))
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Control Modes: 8N1, kein Parity, etc.
    tty.c_cflag &= ~PARENB;          // Parität aus
    tty.c_cflag &= ~CSTOPB;          // 1 Stoppbit
    tty.c_cflag &= ~CSIZE;           // Bits pro Byte ...
    tty.c_cflag |=  CS8;             // ... auf 8 Bits setzen
    tty.c_cflag &= ~CRTSCTS;         // Keine Hardware-Flow-Control
    tty.c_cflag |=  CREAD | CLOCAL;  // Receiver einschalten, Local Mode

    // Input Modes
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // keine Software-Flow-Control

    // Output Modes
    tty.c_oflag &= ~OPOST; // raw output

    // Einstellungen aktivieren
    tcsetattr(fd, TCSANOW, &tty);

    // 3) Daten kontinuierlich einlesen
    std::cout << "Reading from " << serialPort << "...\n";
    while (true) {
        char buffer[256];
        memset(buffer, 0, sizeof(buffer));

        // read(...) liest „irgendeine“ Anzahl von Bytes (bis zu 255) vom Port
        int n = read(fd, &buffer, sizeof(buffer) - 1);

        if (n > 0) {
            // Rohdaten als String ausgeben
            std::string data(buffer, n);

            // Zeilenumbruch entfernen, falls Arduino \r\n sendet
            // (je nach Bedarf z. B. std::getline()-Logik implementieren)
            if (!data.empty() && data.back() == '\n') {
                data.pop_back();
            }
            if (!data.empty() && data.back() == '\r') {
                data.pop_back();
            }

            std::cout << "Empfangen: " << data << std::endl;
        }

        // Kurze Pause, um CPU-Last zu vermeiden
        usleep(10000); // 10 ms
    }

    // 4) Port schließen (im Normalfall nicht erreicht wegen while(true))
    close(fd);
    return 0;
}
