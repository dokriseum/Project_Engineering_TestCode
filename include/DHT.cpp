#include <wiringPi.h>
#include <unistd.h>
#include "DHT.h"

DHT::DHT(uint8_t pin, uint8_t type, uint8_t count) {
  (void)count; // Workaround to avoid compiler warning.
  _pin = pin;
  _type = type;
  _maxcycles = 1000; // 1 millisecond timeout for reading pulses.
}

void DHT::begin(uint8_t usec) {
  pinMode(_pin, INPUT);
  pullUpDnControl(_pin, PUD_UP);
  _lastreadtime = millis() - 2000; // Ensure first read is valid.
  pullTime = usec;
}

float DHT::readTemperature(bool S, bool force) {
  float f = NAN;
  if (read(force)) {
    switch (_type) {
    case DHT11:
      f = data[2];
      break;
    case DHT22:
    case DHT21:
      f = ((data[2] & 0x7F) << 8 | data[3]) * 0.1;
      if (data[2] & 0x80) {
        f *= -1;
      }
      break;
    }
    if (S) {
      f = convertCtoF(f);
    }
  }
  return f;
}

float DHT::readHumidity(bool force) {
  float f = NAN;
  if (read(force)) {
    switch (_type) {
    case DHT11:
      f = data[0];
      break;
    case DHT22:
    case DHT21:
      f = ((data[0]) << 8 | data[1]) * 0.1;
      break;
    }
  }
  return f;
}

float DHT::convertCtoF(float c) {
  return c * 1.8 + 32;
}

float DHT::convertFtoC(float f) {
  return (f - 32) * 0.55555;
}

bool DHT::read(bool force) {
  uint32_t currenttime = millis();
  if (!force && ((currenttime - _lastreadtime) < 2000)) {
    return _lastresult; // Return last valid result.
  }
  _lastreadtime = currenttime;

  // Reset data array.
  for (int i = 0; i < 5; i++) {
    data[i] = 0;
  }

  // Send start signal.
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  usleep(20000); // 20ms for start signal
  digitalWrite(_pin, HIGH);
  usleep(pullTime);
  pinMode(_pin, INPUT);

  // Wait for response from the sensor.
  for (int i = 0; i < 80; i += 2) {
    if (expectPulse(LOW) == 0 || expectPulse(HIGH) == 0) {
      _lastresult = false;
      return _lastresult;
    }
  }

  // Read 40 bits (5 bytes).
  for (int i = 0; i < 40; ++i) {
    uint32_t lowCycles = expectPulse(LOW);
    uint32_t highCycles = expectPulse(HIGH);

    if (lowCycles == 0 || highCycles == 0) {
      _lastresult = false;
      return _lastresult;
    }
    data[i / 8] <<= 1;
    if (highCycles > lowCycles) {
      data[i / 8] |= 1;
    }
  }

  // Verify checksum.
  if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    _lastresult = true;
    return _lastresult;
  } else {
    _lastresult = false;
    return _lastresult;
  }
}

uint32_t DHT::expectPulse(bool level) {
  uint32_t count = 0;
  while (digitalRead(_pin) == level) {
    if (++count >= _maxcycles) {
      return 0; // Timeout
    }
  }
  return count;
}